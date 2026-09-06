# SPDX-License-Identifier: MPL-2.0
# SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
# Author: Vamsi Kalagaturu

"""Exercise annotation and inspection workflows through the served browser UI."""

import json
import os
import shutil

import pytest
from test_dashboard_api import dashboard  # noqa: F401 -- shared HTTP fixture
from test_dashboard_runs import _rec


def test_organize_annotate_and_resume(dashboard):  # noqa: F811 -- pytest fixture
    """Pinning, notes, baseline selection and plot restoration survive navigation/reload."""
    playwright = pytest.importorskip("playwright.sync_api")
    executable = os.environ.get("MOTION_SPEC_BROWSER_EXECUTABLE")
    if not executable:
        pytest.skip("set MOTION_SPEC_BROWSER_EXECUTABLE to run browser integration checks")
    (dashboard.run / "rec.ld.json").write_text(json.dumps(_rec("CompletedRun")))
    os.utime(dashboard.run / "logs/frame_log.pb", (1, 1))
    second = dashboard.run.parent.parent.parent / "20260906T100000Z"
    shutil.copytree(dashboard.run.parent.parent, second)
    with playwright.sync_playwright() as driver:
        browser = driver.chromium.launch(executable_path=executable, headless=True)
        page = browser.new_page(viewport={"width": 1440, "height": 1000})
        errors = []
        page.on("pageerror", lambda error: errors.append(str(error)))
        page.route("**/api/events", lambda route: route.fulfill(status=200, content_type="text/event-stream", body=": idle\n\n"))
        page.goto(dashboard.host)
        page.locator("#search").fill("demo")
        playwright.expect(page.locator("#browser .item:visible")).to_have_count(2)
        page.locator("#browser .item").last.click()
        editor = page.locator(".annotation-editor")
        editor.locator('[name="label"]').fill("Contact baseline")
        editor.locator('[name="tags"]').fill("contact, tuned")
        editor.get_by_role("button", name="Save", exact=True).click()
        playwright.expect(editor.get_by_role("status")).to_have_text("Saved")
        editor.get_by_role("button", name="☆ Pin", exact=True).click()
        playwright.expect(editor.get_by_role("button", name="★ Pinned", exact=True)).to_be_visible()
        page.locator("#generation-filter").select_option("pinned")
        playwright.expect(page.locator("#browser .item:visible")).to_have_count(1)
        page.get_by_text("Generation notes", exact=True).click()
        notes = page.locator(".generation-notes")
        notes.locator(".note-text").fill("Stable contact settings")
        notes.locator(".note-tags").fill("verified")
        notes.get_by_role("button", name="add note", exact=True).click()
        playwright.expect(notes.locator(".note-body")).to_have_text("Stable contact settings")
        page.reload()
        playwright.expect(page.locator('.annotation-editor [name="label"]')).to_have_value("Contact baseline")
        page.get_by_text("Generation notes", exact=True).click()
        playwright.expect(page.locator(".generation-notes .note-body")).to_have_text("Stable contact settings")
        page.locator(".runs .run strong").first.click()
        page.get_by_role("button", name="Use as model baseline", exact=True).click()
        playwright.expect(page.get_by_role("button", name="Clear model baseline", exact=True)).to_be_visible()
        page.locator('[data-panel="notes"]').click()
        notes = page.locator("#panel-notes")
        notes.locator(".note-text").fill("Inspect contact here")
        notes.get_by_role("button", name="Attach current frame", exact=True).click()
        notes.get_by_role("button", name="add note", exact=True).click()
        playwright.expect(notes.locator(".note-body")).to_have_text("Inspect contact here")
        notes.get_by_role("button", name="Frame 0", exact=True).click()
        playwright.expect(page.locator("#panel-plots")).to_be_visible()
        playwright.expect(page.locator(".marker-note")).to_have_count(1)
        page.get_by_role("button", name="Add empty plot", exact=True).click()
        page.locator(".plot-card .signal-menu summary").click()
        page.locator(".signal-option").first.click()
        playwright.expect(page.locator(".plot-card .plot-signal")).to_have_count(1)
        page.get_by_role("textbox", name="Plot preset name", exact=True).fill("Contact view")
        page.get_by_role("button", name="Save plots", exact=True).click()
        playwright.expect(page.get_by_label("Plot preset", exact=True)).to_have_value("Contact view")
        page.reload()
        playwright.expect(page.locator(".plot-card .plot-signal")).to_have_count(1)
        page.locator(".remove-plot").click()
        page.get_by_label("Plot preset", exact=True).select_option("Contact view")
        page.get_by_role("button", name="Load", exact=True).click()
        playwright.expect(page.locator(".plot-card .plot-signal")).to_have_count(1)
        page.screenshot(path=str(dashboard.root / "inspection.png"), full_page=True)
        page.get_by_title("Back to generation", exact=True).click()
        page.locator("#generation-filter").select_option("all")
        page.locator("#browser .item").filter(has_text="20260906T100000Z").click()
        page.locator(".runs .run strong").first.click()
        page.locator('[data-panel="reports"]').click()
        page.get_by_role("button", name="Compare against baseline", exact=True).click()
        playwright.expect(page.locator(".compare-pick")).not_to_have_value("")
        playwright.expect(page.locator(".compare-slot table")).to_be_visible()
        page.get_by_title("Back to generation", exact=True).click()
        page.screenshot(path=str(dashboard.root / "generation.png"), full_page=True)
        page.locator("#browser .item").filter(has_text="20260906T100000Z").click(modifiers=["Control"])
        page.locator("#delete-selected").click()
        playwright.expect(page.locator("dialog.ask")).to_contain_text("1 runs")
        page.locator("dialog.ask").get_by_role("button", name="Move to Trash", exact=True).click()
        playwright.expect(page.locator("#browser .item")).to_have_count(1)
        assert not second.exists()
        page.get_by_role("button", name="Trash", exact=True).click()
        page.get_by_role("searchbox", name="Filter trashed paths", exact=True).fill("20260906T100000Z")
        page.locator(".trash-entries").get_by_role("button", name="Restore", exact=True).click()
        playwright.expect(page.locator(".trash-browser [role=status]")).to_contain_text("Restored")
        assert second.is_dir()
        playwright.expect(page.locator("#browser .item")).to_have_count(2)
        assert errors == []
        browser.close()
