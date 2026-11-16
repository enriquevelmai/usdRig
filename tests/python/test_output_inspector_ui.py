#!/usr/bin/env python
"""
The execution stack panel's output inspector, driven on its real widgets.

This drives plugin/rigExecUsdview/execStackUI.ExecStackPanel itself, on Qt's
offscreen platform, over the rig test_output_inspector_model authors: the
output chooser, the frame field, the lines the panel paints, the stack row
and the viewport pick that name a request, the panel's own tick, and its
teardown. The usdview api is a stand-in carrying the attributes the panel
reads, and its playhead refuses to be moved, so a reach for the viewer's
clock fails at the line that reaches.

What a demanded frame answers is pinned in test_output_inspector_model; this
test pins that the panel asks for one exact frame, paints the disposition it
is handed, and lets go of every demand when its stage or its window goes
away.

A live usdview window needs a display and a GPU, which this host has
neither of: dock placement and the GL viewport beside the panel stay
unproven here.

Usage: test_output_inspector_ui.py
"""
import os
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Sdf, Usd, UsdGeom  # noqa: E402
from pxr.Usdviewq.qt import QtCore, QtWidgets  # noqa: E402

import execStackUI  # noqa: E402
import outputInspectorModel as insp  # noqa: E402
import rigexec  # noqa: E402
from test_output_inspector_model import (  # noqa: E402
    BODY_PATH, DATA_PATH, EMPTY_PATH, POINTS_TARGET, SCALAR_TARGET,
    SHOWN_FRAME, TZ_SAMPLES, _author_empty_rig, _author_rig)

STATUS_WORDS = set(insp.STATUS_LABELS.values())

# The widgets need an application before any of them exists; offscreen keeps
# it from asking for a display.
_APP = QtWidgets.QApplication.instance() or QtWidgets.QApplication(
    ["test_output_inspector_ui"])


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, eps=1e-4):
    return abs(a - b) < eps


def _Sample(attr, frame):
    return float(attr.Get(Usd.TimeCode(frame)))


class _Signal(object):
    """A usdview signal stand-in: connect, emit, and a record of who asked."""

    def __init__(self):
        self.slots = []

    def connect(self, slot):
        self.slots.append(slot)

    def emit(self, *args):
        for slot in self.slots:
            slot(*args)


class _Data(object):
    """The parts of usdview's data model this panel reads.

    `currentFrame` is read-only from the panel's side: its setter records the
    attempt and raises, so a playhead reach fails at the line that reaches
    rather than at a later assertion. `ShowFrame` is the artist moving it,
    which is not the panel's doing.
    """

    def __init__(self, stage):
        self.stage = stage
        self._frame = SHOWN_FRAME
        self.frameWrites = []
        self.selection = _Selection()
        self.signalStageReplaced = _Signal()

    @property
    def currentFrame(self):
        return self._frame

    @currentFrame.setter
    def currentFrame(self, value):
        self.frameWrites.append(value)
        raise AssertionError("the panel moved the playhead to %r" % (value,))

    def ShowFrame(self, value):
        self._frame = value


class _Selection(object):
    def __init__(self):
        self.setPrims = []
        self.signalPrimSelectionChanged = _Signal()

    def setPrim(self, prim):
        self.setPrims.append(str(prim.GetPath()))


class _Api(object):
    """A usdview api with the three things this panel touches."""

    def __init__(self, stage):
        self.qMainWindow = QtWidgets.QMainWindow()
        self.dataModel = _Data(stage)

    def SetStage(self, stage):
        self.dataModel.stage = stage


def _StackRow(panel, suffix):
    """The execution-stack line whose prim path ends with `suffix`."""
    for group in range(panel._tree.topLevelItemCount()):
        node = panel._tree.topLevelItem(group)
        for child in range(node.childCount()):
            item = node.child(child)
            if str(item.data(0, QtCore.Qt.UserRole) or "").endswith(suffix):
                return item
    return None


def _Lines(panel):
    """What the inspector lines actually show, column by column."""
    lines = []
    for index in range(panel._inspectorTree.topLevelItemCount()):
        item = panel._inspectorTree.topLevelItem(index)
        lines.append([item.text(col) for col in range(7)])
    return lines


def _Chooser(panel):
    return [panel._outputBox.itemText(i)
            for i in range(panel._outputBox.count())]


def _WaitOnPanelTimer(panel, deadlineSeconds=15.0):
    """Let the panel's own timer deliver the answer: no manual tick here.

    Nothing pumps the service from the test; the only path to an answer is
    the panel arming its timer on the demand, ticking, reading the service
    and repainting. Whether the pool beat the first read is timing, so the
    wait does not assert the timer was seen running -- only that this path
    needs no hand-driven pump, and that it stops once nothing is owed.
    """
    app = QtWidgets.QApplication.instance()
    deadline = time.monotonic() + deadlineSeconds
    while time.monotonic() < deadline:
        rows = panel._inspector.Rows()
        if rows and all(row.status == insp.READY for row in rows):
            _Check(not panel._inspectorTimer.isActive(),
                   "a settled request kept the panel ticking")
            _Check(panel._inspectorTree.topLevelItemCount() == len(rows),
                   "the answer never reached the screen")
            return rows
        app.processEvents()
    _Check(False, "the panel's own tick never answered: %r"
           % [(row.target, row.phase, row.status, row.reason)
              for row in panel._inspector.Rows()])


def _TickUntilAnswered(panel, deadlineSeconds=15.0):
    """Drive the tick by hand, the way a host with its own clock would."""
    app = QtWidgets.QApplication.instance()
    deadline = time.monotonic() + deadlineSeconds
    while True:
        panel.PollTick()
        if app:
            app.processEvents()
        rows = panel._inspector.Rows()
        if rows and all(row.status == insp.READY for row in rows):
            _Check(not panel._inspectorTimer.isActive(),
                   "a settled request kept the panel ticking")
            return rows
        if not panel._inspector.Selected():
            _Check(False, "an empty selection was waited on")
        if time.monotonic() >= deadline:
            _Check(False, "the panel never answered: %r"
                   % [(row.target, row.phase, row.status, row.reason)
                      for row in rows])


def _EmptyStage():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Scope.Define(stage, "/Nothing")
    return stage


def main():
    stage, rig, tzAttr, _tyAttr, _liftPath, movePath = _author_rig(rigexec)
    authored = stage.GetSessionLayer().ExportToString()

    api = _Api(stage)
    panel = execStackUI.ExecStackPanel(api)
    panel.show()
    QtWidgets.QApplication.instance().processEvents()
    _Check(panel.isVisible(), "the offscreen panel would not show")
    data = api.dataModel

    # The panel wired itself to the host's lifecycle rather than to a copy
    # of it: a replaced stage and a viewport pick both reach it.
    _Check(len(data.signalStageReplaced.slots) == 1,
           "the panel did not subscribe to stage replacement")
    _Check(len(data.selection.signalPrimSelectionChanged.slots) == 1,
           "the panel did not subscribe to viewport selection")

    panel.Rebuild()
    _Check(panel._rig is not None, panel._status.text())
    _Check(panel._inspector.attached, "a compiled rig left it detached")
    _Check(panel._inspector.stats()["subscriptions"] == 0,
           "attaching demanded something on its own")
    _Check(panel._inspectorTree.topLevelItemCount() == 0,
           "attaching painted a line nobody asked for")
    bindings = panel._inspector.Bindings()
    _Check(bindings and panel._outputBox.count() == len(bindings),
           (panel._outputBox.count(), len(bindings)))
    # A mover's own pass is listed under the mover that produced it, beside
    # the target it wrote: an intermediate is nameable from the chooser.
    _Check(any("Body.points" in text and "chain/move" in text
               for text in _Chooser(panel)), _Chooser(panel))
    _Check(any("Body.points" in text and "final" in text
               for text in _Chooser(panel)), _Chooser(panel))

    # -- picking the mover's row asks for that mover's own pass ----------
    stackRow = _StackRow(panel, "/chain/move")
    _Check(stackRow is not None, "the mover is not in the execution stack")
    moverPath = str(stackRow.data(0, QtCore.Qt.UserRole))
    _Check(moverPath == movePath, (moverPath, movePath))
    panel._tree.setCurrentItem(stackRow)
    QtWidgets.QApplication.instance().processEvents()
    selected = panel._inspector.Selected()
    _Check(selected, "picking the mover's row demanded nothing")
    _Check([panel._inspector.Binding(key).phase for key in selected]
           == [moverPath],
           [panel._inspector.Binding(key).phase for key in selected])
    # The row also selects its prim, as it always has; the inspector follows
    # that pick instead of reaching into the viewer's selection itself.
    _Check(data.selection.setPrims == [moverPath], data.selection.setPrims)
    _Check(len(_Chooser(panel)) == 1 and "chain/move" in _Chooser(panel)[0],
           _Chooser(panel))

    # -- an exact absolute frame, typed into the panel -------------------
    panel._frameEdit.setText("2.5")
    panel._ApplyFrameEdit()
    _Check(panel._inspector.Frame() == 2.5, panel._inspector.FrameText())
    _Check(panel._frameEdit.text() == "2.5", panel._frameEdit.text())
    _Check(data.currentFrame == SHOWN_FRAME, "typing a frame moved the frame")
    _Check(data.frameWrites == [], data.frameWrites)

    rows = _WaitOnPanelTimer(panel)
    _Check(len(rows) == 1, [(row.phase, row.status) for row in rows])
    # The panel's own timer drove the read: a tick, not a manual pump.
    _Check(all(row.time == 2.5 and row.frameNamed for row in rows),
           [(row.time, row.frameNamed) for row in rows])
    lines = _Lines(panel)
    _Check(len(lines) == len(rows), (lines, len(rows)))
    for line, row in zip(lines, rows):
        _Check(line[2] == "chain/move", ("the phase column", line))
        _Check(line[3] == "2.5", ("the frame column", line))
        _Check(line[4] == insp.StatusLabel(row.status)
               and line[4] in STATUS_WORDS,
               ("the status column is not the disposition", line))
        _Check(line[5] == row.GenerationText(), ("the generation", line))
        _Check(line[6] == row.ValueText()
               and line[6] not in STATUS_WORDS, ("the value column", line))
    # The shown frame sits past the control's last knot, so its value and
    # frame 2.5's are different numbers: the points the panel shows answer
    # 2.5, not the frame usdview is on.
    atFrame = _Sample(tzAttr, 2.5)
    shownValue = _Sample(tzAttr, SHOWN_FRAME)
    _Check(_Close(atFrame, 1.25) and _Close(shownValue, TZ_SAMPLES[10.0])
           and atFrame != shownValue, (atFrame, shownValue))
    first = rows[0].packet.value.point(0)
    _Check(_Close(first[2], atFrame), (first, atFrame))
    # The painted line spells the payload's count beside its first point.
    painted = _Lines(panel)[0][6]
    _Check(painted == rows[0].ValueText(), (painted, rows[0].ValueText()))
    _Check(painted == "%d pts, first (%.4f, %.4f, %.4f)"
           % (rows[0].packet.value.elements, first[0], first[1], first[2]),
           painted)

    # -- the shown frame: read from the viewer, never written to it ------
    data.ShowFrame(7.0)
    panel._InspectShownFrame()
    _Check(panel._inspector.Frame() == 7.0, panel._inspector.FrameText())
    _Check(panel._frameEdit.text() == "7.0", panel._frameEdit.text())
    _Check(data.currentFrame == 7.0 and data.frameWrites == [],
           data.frameWrites)
    rows = _TickUntilAnswered(panel)
    _Check(all(row.time == 7.0 for row in rows), [row.time for row in rows])
    atFrame = _Sample(tzAttr, 7.0)
    _Check(_Close(atFrame, 3.5), atFrame)
    _Check(_Close(rows[0].packet.value.point(0)[2], atFrame),
           rows[0].ValueText())

    # -- a hidden panel skips its tick, and catches up when shown again --
    panel.hide()
    panel._inspector.DemandFrame(4.0)
    _Check(panel.PollTick() is False, "a hidden panel read the service")
    _Check(not panel._inspectorTimer.isActive(),
           "a hidden panel kept ticking")
    panel.show()
    QtWidgets.QApplication.instance().processEvents()
    _Check(panel._inspector.Frame() == 4.0, panel._inspector.FrameText())
    rows = _TickUntilAnswered(panel)
    _Check(all(row.time == 4.0 for row in rows), [row.time for row in rows])

    # -- a prim picked in the viewport names the request -----------------
    # The selection data model emits two sets of Sdf.Path, never prims, so
    # that is exactly what the stand-in emits. Clearing first makes the pick's
    # effect the only thing that can fill the selection.
    panel._inspector.Select([])
    data.selection.signalPrimSelectionChanged.emit(
        {Sdf.Path(DATA_PATH)}, set())
    scalarKeys = panel._inspector.Selected()
    _Check(scalarKeys,
           "a viewport pick left the inspector with no selection")
    _Check([panel._inspector.Binding(key).target for key in scalarKeys]
           == [SCALAR_TARGET] * len(scalarKeys),
           [panel._inspector.Binding(key).target for key in scalarKeys])
    # A blank frame field is the default frame, spelled as the default.
    panel._frameEdit.setText("")
    panel._ApplyFrameEdit()
    _Check(panel._inspector.Frame() is None
           and panel._inspector.FrameText() == "default",
           panel._inspector.FrameText())
    rows = _TickUntilAnswered(panel)
    _Check(rows and all(row.kind == "scalar" for row in rows),
           [(row.kind, row.status) for row in rows])
    for line in _Lines(panel):
        _Check(line[3] == "default", line)
    final = [row for row in rows if row.phase == "final"]
    _Check(len(final) == 1, [(row.phase, row.status) for row in rows])
    live = float(rig.evaluate().moved_property(SCALAR_TARGET))
    _Check(_Close(float(final[0].packet.value.scalar), live),
           (final[0].ValueText(), live))
    # Picking another output's prim replaces the selection with that output's
    # exact handles: the panel follows the signal rather than keeping its
    # last pick, and the new pick is demanded at the frame already shown.
    data.selection.signalPrimSelectionChanged.emit(
        {Sdf.Path(BODY_PATH)}, {Sdf.Path(DATA_PATH)})
    bodyKeys = panel._inspector.Selected()
    _Check(bodyKeys, "picking the body prim left no selection")
    _Check(set(bodyKeys) != set(scalarKeys),
           "picking another output answered the same selection")
    _Check([panel._inspector.Binding(key).target for key in bodyKeys]
           == [POINTS_TARGET] * len(bodyKeys),
           [panel._inspector.Binding(key).target for key in bodyKeys])
    bodyRows = _TickUntilAnswered(panel)
    _Check(bodyRows and all(row.kind == "points" for row in bodyRows),
           [(row.kind, row.status) for row in bodyRows])
    # A prim with nothing exported says so instead of emptying the panel.
    panel._InspectPath("/Model/Geom/NoSuchPrim")
    _Check("writes no exported output" in panel._inspectorStatus.text(),
           panel._inspectorStatus.text())
    _Check(panel._inspector.Selected() == [], panel._inspector.Selected())

    # -- and a real request again, so the teardown has something to end --
    panel._InspectPath(DATA_PATH)
    panel._frameEdit.setText("2")
    panel._ApplyFrameEdit()
    rows = _TickUntilAnswered(panel)
    live = float(rig.evaluate(2.0).moved_property(SCALAR_TARGET))
    final = [row for row in rows if row.phase == "final"]
    _Check(_Close(float(final[0].packet.value.scalar), live),
           (final[0].ValueText(), live))
    for line in _Lines(panel):
        _Check(line[3] == "2", line)

    # -- the stage changed under the panel: the old demand ends ----------
    service = panel._inspector._service
    before = service.stats()
    _Check(before["subscriptions"] == 1 and before["pinned_payloads"] > 0,
           before)
    api.SetStage(_EmptyStage())
    data.signalStageReplaced.emit()
    after = service.stats()
    _Check(after["subscriptions"] == 0 and after["demands_in_flight"] == 0
           and after["pinned_payloads"] == 0 and after["pinned_bytes"] == 0,
           after)
    _Check(not panel._inspector.attached,
           "a replaced stage left a subscription open")
    _Check(panel._inspectorTree.topLevelItemCount() == 0,
           "a replaced stage left lines on screen")
    _Check(panel._outputBox.count() == 0, panel._outputBox.count())
    _Check("No RigExecRoot" in panel._status.text(), panel._status.text())

    # -- a payload with no points paints a count, not an aborted repaint ---
    emptyStage, _emptyRig = _author_empty_rig(rigexec)
    emptyAuthored = emptyStage.GetSessionLayer().ExportToString()
    api.SetStage(emptyStage)
    panel.Rebuild()
    _Check(panel._inspector.attached, panel._status.text())
    panel._InspectPath(EMPTY_PATH)
    emptyRows = _TickUntilAnswered(panel)
    _Check(emptyRows and all(row.kind == "points" and row.ready
                             for row in emptyRows),
           [(row.kind, row.status, row.reason) for row in emptyRows])
    _Check(all(row.packet.value.elements == 0 for row in emptyRows),
           [row.ValueText() for row in emptyRows])
    emptyLines = _Lines(panel)
    _Check(len(emptyLines) == len(emptyRows), (emptyLines, len(emptyRows)))
    for line, row in zip(emptyLines, emptyRows):
        # The value column answers with the count: a zero-point payload has
        # no first point to sample, and the repaint has to finish anyway.
        _Check(line[6] == row.ValueText(), ("the value column", line))
        _Check(line[6] == "0 pts", line)
        _Check(line[4] == insp.StatusLabel(insp.READY), line)
    _Check(emptyStage.GetSessionLayer().ExportToString() == emptyAuthored,
           "inspecting an empty payload authored into the stage")

    # -- closing the panel closes every demand it holds ------------------
    api.SetStage(stage)
    panel.Rebuild()
    _Check(panel._inspector.attached, "the panel did not re-attach")
    panel._InspectAllOutputs()
    _TickUntilAnswered(panel)
    held = panel._inspector._service
    _Check(held.stats()["pinned_payloads"] > 0, held.stats())
    panel.close()
    stats = held.stats()
    _Check(stats["subscriptions"] == 0 and stats["pinned_payloads"] == 0
           and stats["pinned_bytes"] == 0, stats)
    _Check(not panel._inspector.attached, "a closed panel kept its service")
    _Check(not panel._inspectorTimer.isActive(), "a closed panel still ticks")
    _Check(panel._inspectorTree.topLevelItemCount() == 0,
           "a closed panel kept its lines")

    # Nothing above authored the stage or moved a playhead.
    _Check(stage.GetSessionLayer().ExportToString() == authored,
           "the inspector authored into the stage")
    _Check(data.frameWrites == [], data.frameWrites)

    print("output inspector panel: real widgets, stack row and viewport pick "
          "naming exact-frame requests, statuses painted from the "
          "disposition, teardown ending every demand -- all checks passed")


if __name__ == "__main__":
    main()
