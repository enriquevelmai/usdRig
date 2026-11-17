# Copyright (c) 2026 Nick Burkard
# SPDX-License-Identifier: MIT
"""Compiled mover order and solver outputs for the selected rig.

Read phases and dependency edges determine which produced values a consumer
reads. The panel displays compiler diagnostics without inferring extra rules
from operation types or their positions in the list.

Selecting a row also inspects what that row writes: the exported output is
demanded at one exact frame through outputInspectorModel, which runs on the
rigexec.subscriptions facade. The panel's own timer is the tick; a host with
a recurring tick of its own can drive PollTick() instead.
"""
import os
import sys

from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    import sessionRegistry
except ImportError:                    # loader that did not add our dir
    import os as _os
    import sys as _sys
    _sys.path.insert(0, _os.path.dirname(_os.path.abspath(__file__)))
    import sessionRegistry

# The inspector model imports rigexec, so it lands the way the panel imports
# rigexec itself: a missing native module leaves the panel open with the
# reason on its status line rather than failing the menu action.
_inspectorImportError = ""
try:
    import outputInspectorModel
except ImportError as _error:
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import outputInspectorModel
    except ImportError as _nested:
        outputInspectorModel = None
        _inspectorImportError = str(_nested) or "outputInspectorModel"

# How often an open panel reads back what the subscription service
# published. The service's own pool does the work; this only repaints, and
# it stops while nothing is owed -- a demand owed again by a stage edit
# re-arms it. A host with a recurring tick of its own can drive PollTick().
_INSPECTOR_POLL_MS = 250


# Schema-type tokens per filter. A type may appear in several groups -- a
# single-chain IK is legitimately both a constraint and IK -- so the
# filters overlap on purpose.
_GROUPS = {
    "All": None,
    "Deformers": ("SkinMover", "MatrixMover", "CurveMover", "LatticeMover",
                  "SurfaceMover", "PatchMover", "SmoothMover", "DeltaMushMover", "WrinkleMover", "BlendShapeMover",
                  "MlMover", "VolumeCorrectMover"),
    "Constraints": ("Constraint",),
    "Solvers": ("FkChain", "TwoBoneIk", "BlendPointFrames",
                "TwistDistribution", "Ribbon", "SplineIk"),
    "IK": ("TwoBoneIk", "SingleChainIk", "SplineIk"),
    "FK": ("FkChain",),
    "Weights": ("Weight",),
    "Movers only": ("Mover",),
}

_SOLVER_TYPES = ("RigExecFkChain", "RigExecTwoBoneIk",
                 "RigExecBlendPointFrames", "RigExecTwistDistribution",
                 "RigExecRibbon", "RigExecSplineIk")
def _tail(path, keep=1):
    parts = str(path).split("/")
    return "/".join(parts[-keep:]) if len(parts) > keep else str(path)


def _OutputLabel(binding):
    """One exported handle as the chooser spells it."""
    return "%s  [%s %s]" % (_tail(binding.target, 2), binding.kind,
                            _PhaseLabel(binding.phase))


def _PhaseLabel(phase):
    """A read phase as the panel spells it.

    A mover's own pass is exported at the phase named by its path, so it
    keeps its last two segments: the chooser and the lines say which mover
    produced the value rather than showing a full stage path.
    """
    text = str(phase)
    return _tail(text, 2) if text.startswith("/") else text


class ExecStackPanel(QtWidgets.QDialog):
    """Dockable-ish dialog listing the execution stack of the open stage."""

    # One panel per usdview session, filed under its main window: several
    # sessions can share this module in one process.
    _sessions = sessionRegistry.SessionRegistry("execution stack panels")

    @classmethod
    def GetInstance(cls, usdviewApi):
        panel = cls._sessions.Get(usdviewApi)
        if panel is None:
            panel = cls._sessions.Set(usdviewApi, cls(usdviewApi))
        else:
            panel._api = usdviewApi
        return panel

    def __init__(self, usdviewApi, parent=None):
        super(ExecStackPanel, self).__init__(
            parent or usdviewApi.qMainWindow)
        self._api = usdviewApi
        self.setWindowTitle("Execution Stack")
        self.resize(920, 640)

        layout = QtWidgets.QVBoxLayout(self)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Rig root:"))
        self._rootBox = QtWidgets.QComboBox()
        self._rootBox.setMinimumWidth(220)
        self._rootBox.currentIndexChanged.connect(lambda _i: self.Rebuild())
        controls.addWidget(self._rootBox)

        controls.addWidget(QtWidgets.QLabel("Show:"))
        self._filterBox = QtWidgets.QComboBox()
        for name in ("All", "Deformers", "Constraints", "Solvers", "IK",
                     "FK", "Weights", "Movers only"):
            self._filterBox.addItem(name)
        self._filterBox.currentIndexChanged.connect(
            lambda _i: self._Repopulate())
        controls.addWidget(self._filterBox)

        controls.addWidget(QtWidgets.QLabel("Find:"))
        self._search = QtWidgets.QLineEdit()
        self._search.setPlaceholderText("name, type or target, e.g. elbow")
        self._search.textChanged.connect(lambda _t: self._Repopulate())
        controls.addWidget(self._search, 1)

        refresh = QtWidgets.QPushButton("Refresh")
        refresh.clicked.connect(self.Rebuild)
        controls.addWidget(refresh)
        layout.addLayout(controls)

        self._tree = QtWidgets.QTreeWidget()
        self._tree.setHeaderLabels(["#", "Type", "Name", "Writes"])
        self._tree.setRootIsDecorated(True)
        self._tree.setAlternatingRowColors(True)
        self._tree.setColumnWidth(0, 44)
        self._tree.setColumnWidth(1, 210)
        self._tree.setColumnWidth(2, 250)
        self._tree.itemSelectionChanged.connect(self._OnSelect)
        layout.addWidget(self._tree, 1)

        self._status = QtWidgets.QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

        self._BuildInspector(layout)

        self._solvers = []
        self._movers = []
        self._rig = None

        # The host's own lifecycle: a replaced stage drops the rig this panel
        # compiled and every demand made from it, and a prim picked in the
        # viewport names the outputs to inspect. Optional as in the other
        # panels: a bare api has neither signal.
        try:
            dataModel = self._api.dataModel
            dataModel.signalStageReplaced.connect(self._OnStageReplaced)
            dataModel.selection.signalPrimSelectionChanged.connect(
                self._OnPrimSelection)
        except Exception:
            pass
        # usdview forwards Usd.Notice.ObjectsChanged on this signal, from the
        # listener its data model registers per stage, so a control edit made
        # anywhere reaches the panel. Optional like the others: a bare api
        # has no signal to hand over.
        try:
            self._api.dataModel.signalPrimsChanged.connect(
                self._OnPrimsChanged)
        except Exception:
            pass

    # -- output inspector ------------------------------------------------

    def _BuildInspector(self, layout):
        """The inspector section: one exported output at one exact frame.

        Every disposition, spelling and color comes from
        outputInspectorModel, the same model the headless test drives; this
        panel lists, demands and repaints.
        """
        group = QtWidgets.QGroupBox("Exported outputs")
        groupLayout = QtWidgets.QVBoxLayout(group)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Inspect:"))
        self._outputBox = QtWidgets.QComboBox()
        self._outputBox.setMinimumWidth(240)
        self._outputBox.currentIndexChanged.connect(
            lambda _i: self._OnOutputChanged())
        row.addWidget(self._outputBox)

        self._allOutputsButton = QtWidgets.QPushButton("All of this rig")
        self._allOutputsButton.clicked.connect(self._InspectAllOutputs)
        row.addWidget(self._allOutputsButton)

        row.addWidget(QtWidgets.QLabel("At frame:"))
        self._frameEdit = QtWidgets.QLineEdit()
        self._frameEdit.setPlaceholderText("blank = default, or -3, 2.5, 0")
        self._frameEdit.returnPressed.connect(self._ApplyFrameEdit)
        row.addWidget(self._frameEdit, 1)

        self._shownFrameButton = QtWidgets.QPushButton("Shown frame")
        self._shownFrameButton.clicked.connect(self._InspectShownFrame)
        row.addWidget(self._shownFrameButton)
        groupLayout.addLayout(row)

        self._inspectorTree = QtWidgets.QTreeWidget()
        self._inspectorTree.setHeaderLabels(
            ["Output", "Kind", "Phase", "Frame", "Status", "Generation",
             "Value"])
        self._inspectorTree.setRootIsDecorated(False)
        self._inspectorTree.setAlternatingRowColors(True)
        groupLayout.addWidget(self._inspectorTree, 1)

        self._inspectorStatus = QtWidgets.QLabel("")
        self._inspectorStatus.setWordWrap(True)
        groupLayout.addWidget(self._inspectorStatus)

        layout.addWidget(group, 1)

        self._inspector = (None if outputInspectorModel is None else
                           outputInspectorModel.OutputInspectorModel(
                               host=self))
        # The panel's own tick, armed while a demand is owed or the stage
        # moved under one: pumping and polling are cheap, and a settled
        # request repaints nothing. A host with a recurring tick can drive
        # PollTick itself.
        self._inspectorTimer = QtCore.QTimer(self)
        self._inspectorTimer.setSingleShot(False)
        self._inspectorTimer.setInterval(_INSPECTOR_POLL_MS)
        self._inspectorTimer.timeout.connect(self.PollTick)
        self._RefreshInspectorStatus()

    def DisplayedTime(self):
        """The frame usdview is showing, handed to the model to READ.

        This is the only way the viewer's clock reaches an inspection, and it
        goes one way: the panel has no path that sets the playhead, so
        inspecting frame 2.5 while the artist sits at 12 asks for 2.5 and
        leaves 12 where it was. Nothing here authors a stage opinion either:
        the demand runs on the subscription service.
        """
        try:
            return self._api.dataModel.currentFrame
        except Exception:
            return None

    def _InspectorReason(self):
        if outputInspectorModel is not None:
            return ""
        return _inspectorImportError or "the inspector model is not importable"

    def _AttachInspector(self, rig):
        """Inspect the rig this panel just compiled."""
        if self._inspector is None:
            return
        # Attach closes the previous service first: its lanes join, and its
        # handles belong to a compile epoch the recompile has already raised.
        self._inspector.Attach(rig)
        self._RepopulateOutputs(self._inspector.Bindings())
        self._PollInspector()

    def _CloseInspector(self):
        """End every demand this panel holds: no subscription, no worker and
        no pinned payload outlives the panel or its stage."""
        if self._inspector is None:
            return
        self._inspector.Close()
        try:
            self._inspectorTimer.stop()
        except Exception:
            pass
        self._RepopulateOutputs([])
        self._RepaintInspector()
        self._RefreshInspectorStatus()

    def _RepopulateOutputs(self, bindings):
        """Refill the output chooser, keeping the same handle when it lives."""
        keep = self._outputBox.currentData()
        self._outputBox.blockSignals(True)
        self._outputBox.clear()
        for binding in bindings:
            self._outputBox.addItem(_OutputLabel(binding),
                                    binding.semantic_key)
        if keep is not None:
            index = self._outputBox.findData(keep)
            if index >= 0:
                self._outputBox.setCurrentIndex(index)
        self._outputBox.blockSignals(False)

    def _CurrentBinding(self):
        if self._inspector is None:
            return None
        key = self._outputBox.currentData()
        if not key:
            return None
        return self._inspector.Binding(key)

    def _OnOutputChanged(self):
        """The chooser narrowed the request to one exported handle."""
        binding = self._CurrentBinding()
        if self._inspector is None or binding is None:
            return
        self._inspector.Select([binding])
        self._PollInspector()

    def _InspectAllOutputs(self):
        if self._inspector is None:
            return
        self._inspector.Select(self._inspector.Bindings())
        self._PollInspector()

    def _InspectPath(self, path):
        """Inspect what the prim at `path` writes, and list only those
        handles in the chooser."""
        if self._inspector is None:
            return
        matches = self._inspector.HandlesFor(path)
        self._RepopulateOutputs(matches)
        self._inspector.Select(matches)
        self._PollInspector()
        if not matches:
            self._inspectorStatus.setText(
                "%s writes no exported output of this rig." % path)

    def _InspectShownFrame(self):
        """Inspect at the frame the viewer shows -- read, never moved."""
        if self._inspector is None:
            return
        self._inspector.DemandDisplayedFrame()
        self._SyncFrameEdit()
        self._PollInspector()

    def _ApplyFrameEdit(self):
        """Inspect at the typed absolute frame; blank is the default frame."""
        if self._inspector is None:
            return
        text = self._frameEdit.text().strip()
        frame = None
        if text:
            try:
                frame = float(text)
            except ValueError:
                self._RefreshInspectorStatus(
                    "%s is not a frame: blank is the default frame, or one "
                    "number such as -3 or 2.5." % text)
                return
        try:
            self._inspector.DemandFrame(frame)
        except ValueError as error:
            self._RefreshInspectorStatus(str(error))
            return
        self._SyncFrameEdit()
        self._PollInspector()

    def _SyncFrameEdit(self):
        frame = self._inspector.Frame()
        text = "" if frame is None else repr(frame)
        if self._frameEdit.text().strip() != text:
            self._frameEdit.setText(text)

    def PollTick(self):
        """One tick of the host's clock: read back what the service published
        and repaint only on news.

        The panel's own timer drives this while it is open; a host with a
        recurring tick can call it directly. A hidden panel skips the read
        and repaints when it is shown again.
        """
        if not self.isVisible():
            return False
        return self._PollInspector()

    def _PollInspector(self):
        if self._inspector is None:
            return False
        changed = self._inspector.Poll()
        if changed:
            self._RepaintInspector()
        self._RefreshInspectorStatus()
        if (self._inspector.attached and self._inspector.HasWork()
                and self.isVisible()):
            self._inspectorTimer.start()
        else:
            self._inspectorTimer.stop()
        return changed

    def _RepaintInspector(self):
        self._inspectorTree.clear()
        if self._inspector is None:
            return
        mono = QtGui.QFont("Consolas")
        mono.setStyleHint(QtGui.QFont.Monospace)
        for row in self._inspector.Rows():
            item = QtWidgets.QTreeWidgetItem(
                [_tail(row.target, 3), row.kind, _PhaseLabel(row.phase),
                 row.frameText, row.label, row.GenerationText(),
                 row.ValueText()])
            for column in range(7):
                item.setFont(column, mono)
            item.setForeground(4, QtGui.QColor(row.color))
            self._inspectorTree.addTopLevelItem(item)
        self._inspectorTree.resizeColumnToContents(0)
        self._inspectorTree.resizeColumnToContents(6)

    def _RefreshInspectorStatus(self, override=""):
        if self._inspector is None:
            self._inspectorStatus.setText(
                "No output inspector: %s." % self._InspectorReason())
            return
        if override:
            self._inspectorStatus.setText(override)
            return
        self._inspectorStatus.setText(self._inspector.StatusText())

    # -- data ------------------------------------------------------------

    def _Stage(self):
        model = getattr(self._api, "dataModel", None)
        return model.stage if model else None

    def _RigRoots(self, stage):
        return [str(p.GetPath()) for p in stage.Traverse()
                if p.GetTypeName() == "RigExecRoot"] if stage else []

    def Rebuild(self):
        """Recompile the rig and re-read its order. Safe to call often."""
        self._solvers, self._movers = [], []
        self._compileError = None
        self._compileDiagnostics = []
        stage = self._Stage()
        if stage is None:
            self._status.setText("No stage open.")
            self._tree.clear()
            self._CloseInspector()
            return

        roots = self._RigRoots(stage)
        current = self._rootBox.currentText()
        self._rootBox.blockSignals(True)
        self._rootBox.clear()
        for r in roots:
            self._rootBox.addItem(r)
        if current in roots:
            self._rootBox.setCurrentIndex(roots.index(current))
        self._rootBox.blockSignals(False)

        if not roots:
            self._status.setText(
                "No RigExecRoot on this stage, so there is no execution "
                "stack to show.")
            self._tree.clear()
            self._CloseInspector()
            return

        root = self._rootBox.currentText() or roots[0]

        # These rows show declared solver outputs, not a separate phase.
        for prim in stage.Traverse():
            t = prim.GetTypeName()
            if t not in _SOLVER_TYPES:
                continue
            if not prim.GetPath().HasPrefix(root):
                continue
            rel = prim.GetRelationship("rigExec:joints")
            claims = [str(x) for x in rel.GetTargets()] if rel else []
            self._solvers.append((t, prim.GetName(),
                                  str(prim.GetPath()), claims))
        self._solvers.sort()

        # The ordered part needs a compile, which can legitimately fail on
        # a rig mid-authoring -- report it rather than throwing into Qt.
        try:
            import rigexec
        except ImportError:
            self._compileError = "The rigexec Python module is not importable."
            self._rig = None
            self._Repopulate()
            self._CloseInspector()
            return
        try:
            rig = rigexec.Rig(stage, root)
            rig.compile()
            order = rig.mover_order
            if callable(order):
                order = order()
            self._movers = list(order)
            self._compileDiagnostics = [
                "Skipped %s: %s" % (path, reason)
                for path, reason in sorted(rig.skipped_operations().items())]
            self._compileError = None
            self._rig = rig
        except Exception as exc:  # compile errors are expected while rigging
            self._movers = []
            self._rig = None
            self._compileError = str(exc).strip() or type(exc).__name__
        self._Repopulate()
        # The exported outputs come off that same compile: a rig that failed
        # has nothing to inspect, and the demand made from its previous
        # program ends here rather than going stale on a poll.
        if self._rig is not None:
            self._AttachInspector(self._rig)
        else:
            self._CloseInspector()

    # -- view ------------------------------------------------------------

    def _Keep(self, schemaType, name, targets):
        tokens = _GROUPS.get(self._filterBox.currentText())
        if tokens is not None:
            if not any(tok.lower() in schemaType.lower() for tok in tokens):
                return False
        needle = self._search.text().strip().lower()
        if needle:
            haystack = " ".join([schemaType, name] +
                                [str(t) for t in targets]).lower()
            if needle not in haystack:
                return False
        return True

    def _Repopulate(self):
        self._tree.clear()
        mono = QtGui.QFont("Consolas")
        mono.setStyleHint(QtGui.QFont.Monospace)

        solverRoot = QtWidgets.QTreeWidgetItem(
            ["", "SOLVERS", "declared outputs", "scheduled by dependency"])
        self._tree.addTopLevelItem(solverRoot)
        shown_solvers = 0
        for t, name, path, claims in self._solvers:
            if not self._Keep(t, name, claims):
                continue
            item = QtWidgets.QTreeWidgetItem(
                ["", t.replace("RigExec", ""), name,
                 "writes %d: %s" % (len(claims),
                                    ", ".join(_tail(c) for c in claims[:6]))])
            item.setData(0, QtCore.Qt.UserRole, path)
            solverRoot.addChild(item)
            shown_solvers += 1
        solverRoot.setExpanded(True)

        moverRoot = QtWidgets.QTreeWidgetItem(
            ["", "EXECUTION ORDER", "movers", "compiled dependency order"])
        self._tree.addTopLevelItem(moverRoot)
        shown_movers = 0
        for e in self._movers:
            t = e["type"]
            name = e["path"].rsplit("/", 1)[-1]
            targets = e.get("targets") or []
            if not self._Keep(t, name, targets):
                continue
            item = QtWidgets.QTreeWidgetItem(
                [str(e["ordinal"]), t.replace("RigExec", ""), name,
                 ", ".join(_tail(x) for x in targets[:3])])
            item.setData(0, QtCore.Qt.UserRole, e["path"])
            for col in range(4):
                item.setFont(col, mono)
            moverRoot.addChild(item)
            shown_movers += 1
        moverRoot.setExpanded(True)

        self._status.setText(self._StatusText(shown_solvers, shown_movers))

    def _StatusText(self, shown_solvers, shown_movers):
        if getattr(self, "_compileError", None):
            return ("Rig compile failed, so there is no mover order: %s"
                    % self._compileError)

        bits = ["%d of %d solvers, %d of %d movers shown"
                % (shown_solvers, len(self._solvers), shown_movers,
                   len(self._movers))]

        bits.extend(getattr(self, "_compileDiagnostics", ()))
        return "  ".join(bits)

    def _OnSelect(self):
        """Selecting a row selects the prim, so the usual panels follow, and
        names the exported outputs to inspect."""
        items = self._tree.selectedItems()
        if not items:
            return
        path = items[0].data(0, QtCore.Qt.UserRole)
        stage = self._Stage()
        if not path or stage is None:
            return
        prim = stage.GetPrimAtPath(str(path).split(".")[0])
        if not prim or not prim.IsValid():
            return
        try:
            self._api.dataModel.selection.setPrim(prim)
        except Exception:
            pass
        self._InspectPath(str(prim.GetPath()))

    # -- lifecycle -------------------------------------------------------

    def _OnStageReplaced(self):
        """The stage changed under the panel: the rig it compiled is gone,
        so every demand made from it ends before the new stage is read."""
        self._CloseInspector()
        self.Rebuild()

    def _OnPrimsChanged(self, _primChange=None, _propertyChange=None):
        """The stage moved under a standing demand: owe one more tick.

        The service reads the live stage in the capture its poll takes, so a
        control edit reaches an answered line only through a poll. The model
        counts the edit as owed work, so the panel's next tick takes that
        capture: it finds the standing record still answers the frame and the
        tick stops again, or demands the frame's new answer. Restarting a
        running timer would push its next tick out, so a burst of edits is
        still read at the panel's own rate, and the inspected frame and the
        viewer's are both left where they were.
        """
        if self._inspector is None or not self.isVisible():
            return
        if not self._inspector.Selected():
            return
        self._inspector.NoteStageEdit()
        if not self._inspectorTimer.isActive():
            self._inspectorTimer.start()

    def _OnPrimSelection(self, added=None, removed=None):
        """A prim picked in the viewport names the outputs to inspect.

        The signal carries two sets of Sdf.Path, not prims
        (Usdviewq/selectionDataModel.py), and its path is what names an
        exported output.
        """
        if not self.isVisible():
            return
        picked = sorted(str(path) for path in (added or ()) if str(path))
        if not picked:
            return
        # The selection never empties, so a pick that leaves only usdview's
        # standing root path inspects the root; otherwise a named prim wins.
        self._InspectPath(next((p for p in picked if p != "/"), picked[0]))

    def showEvent(self, event):
        # A hidden panel skipped its ticks: catch up on what the service
        # published and re-arm the tick if a demand is still owed.
        super(ExecStackPanel, self).showEvent(event)
        self._PollInspector()

    def hideEvent(self, event):
        super(ExecStackPanel, self).hideEvent(event)
        try:
            self._inspectorTimer.stop()
        except Exception:
            pass

    def closeEvent(self, event):
        """Panel teardown closes the subscriptions it holds: no demand, no
        pool and no pinned payload outlives the panel."""
        self._CloseInspector()
        self._rig = None
        super(ExecStackPanel, self).closeEvent(event)


def OpenExecStackPanel(usdviewApi):
    panel = ExecStackPanel.GetInstance(usdviewApi)
    panel.Rebuild()
    panel.show()
    panel.raise_()
    panel.activateWindow()
    return panel
