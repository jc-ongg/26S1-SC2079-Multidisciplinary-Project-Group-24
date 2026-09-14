// Run with Playwright available on NODE_PATH and the simulator at SIMULATOR_URL.
// API fixtures exercise the existing /path contract without starting YOLO or robot services.
const { chromium } = require("playwright");
const assert = require("node:assert/strict");
const fs = require("node:fs");

(async () => {
  const browser = await chromium.launch({ headless: true, channel: process.env.BROWSER_CHANNEL || "chrome" });
  const page = await browser.newPage({ viewport: { width: 1440, height: 1150 } });
  const errors = [];
  page.on("pageerror", (error) => errors.push(error.message));
  const click = (name) => page.getByRole("button", { name, exact: true }).click();
  const count = (selector) => page.locator(selector).count();
  const waitText = (text) => page.getByText(text, { exact: true }).waitFor();
  const fixture = { data: {
    path: [{ x: 1, y: 1, d: 0, s: -1 }, { x: 1, y: 5, d: 0, s: 2 }, { x: 5, y: 7, d: 2, s: -1 }, { x: 8, y: 7, d: 2, s: 3 }],
    commands: ["FW04", "SNAP2_C", "FR01", "FW03", "SNAP3", "FIN"], distance: 38,
  }, error: null };
  let mode = "ok";
  let release;
  await page.route("**/path", async (route) => {
    assert.match(route.request().postData(), /^STATUS,OBSTACLE_LIST,false,1;1;NORTH/);
    if (mode === "delay") await new Promise((resolve) => { release = resolve; });
    if (mode === "error") return route.fulfill({ status: 500, body: "test failure" });
    await route.fulfill({ contentType: "application/json", body: JSON.stringify(fixture) });
  });
  try {
    const response = await page.goto(process.env.SIMULATOR_URL || "http://127.0.0.1:3000");
    assert.equal(response.status(), 200);
    await page.getByRole("heading", { name: "Algorithm simulator" }).waitFor();
    await page.getByLabel("Image facing direction").selectOption("2");
    await click("Add obstacle at (5, 5)");
    await page.getByRole("button", { name: "Obstacle #1 at (5, 5), facing East", exact: true }).waitFor();
    await click("Obstacle #1 at (5, 5), facing East");
    await page.getByRole("button", { name: "Obstacle #1 at (5, 5), facing South", exact: true }).waitFor();
    await waitText("Rotated obstacle #1 clockwise to face South.");
    await click("Add obstacle at (1, 1)");
    await waitText("An obstacle cannot overlap the robot footprint.");
    await click("Add obstacle at (1, 9)");
    await click("Remove obstacle 1");
    await page.getByLabel("Obstacle X", { exact: true }).fill("12");
    await page.getByLabel("Obstacle Y", { exact: true }).fill("7");
    await click("Add obstacle");
    await page.getByRole("button", { name: "Obstacle #3 at (12, 7), facing East", exact: true }).waitFor();
    await click("Add obstacle at (18, 18)");
    await click("Compute route");
    await waitText("Movement 0 / 3");
    assert.equal(await count('path[class*="planned"]'), 3);
    assert.equal(await count('path[class*="trace"]'), 0);
    await page.getByLabel("Visit 1: obstacle 2", { exact: true }).waitFor();
    await waitText("Not in route: #4");
    await click("Next →");
    await page.getByLabel("Visit 1: obstacle 2, completed", { exact: true }).waitFor();
    assert.equal(await count('path[class*="trace"]'), 1);
    await click("← Previous");
    assert.equal(await count('path[class*="trace"]'), 0);
    await page.getByLabel("Visit 1: obstacle 2", { exact: true }).waitFor();
    await click("Play");
    await waitText("Movement 1 / 3");
    await click("Pause");
    const paused = await page.locator('path[class*="trace"]').count();
    await page.waitForTimeout(650);
    assert.equal(await count('path[class*="trace"]'), paused);
    await click("Play");
    await waitText("Movement 3 / 3");
    await page.getByLabel("Visit 2: obstacle 3, completed", { exact: true }).waitFor();
    assert.equal(await count('path[class*="trace"]'), 3);
    fs.mkdirSync("test-results", { recursive: true });
    await page.screenshot({ path: "test-results/simulator-desktop.png", fullPage: true });
    await page.setViewportSize({ width: 390, height: 844 });
    await page.screenshot({ path: "test-results/simulator-mobile.png", fullPage: true });
    assert.equal(await page.evaluate(() => document.documentElement.scrollWidth > innerWidth), false);
    await page.setViewportSize({ width: 1440, height: 1150 });
    await click("Replay");
    await click("Reset Robot");
    await page.waitForTimeout(650);
    await waitText("Movement 0 / 0");
    assert.equal(await count('path[class*="trace"]'), 0);
    assert.equal(await count('path[class*="planned"]'), 0);
    assert.equal(await page.getByRole("button", { name: /^Remove obstacle/ }).count(), 3);
    mode = "delay";
    await click("Compute route");
    await page.waitForTimeout(100);
    assert.ok(release, "pending request intercepted");
    await click("Reset All");
    release();
    await page.waitForTimeout(250);
    await waitText("Movement 0 / 0");
    assert.equal(await count('path[class*="planned"]'), 0);
    await click("Add obstacle at (19, 19)");
    await page.getByRole("button", { name: "Obstacle #1 at (19, 19), facing East", exact: true }).waitFor();
    mode = "error";
    await click("Compute route");
    await page.getByRole("alert").filter({ hasText: "Route request failed" }).waitFor();
    assert.equal(await page.getByRole("button", { name: "Compute route", exact: true }).isEnabled(), true);
    mode = "ok";
    await click("Compute route");
    await waitText("Movement 0 / 3");
    await click("Add obstacle at (17, 19)");
    await waitText("Movement 0 / 0");
    assert.equal(await count('path[class*="planned"]'), 0);
    await click("Reset All");
    for (let x = 0; x < 11; x++) await click(`Add obstacle at (${x}, 19)`);
    await page.getByRole("button", { name: "Obstacle #11 at (10, 19), facing East", exact: true }).waitFor();
    assert.deepEqual(errors, []);
    console.log("PASS: placement, duplicate/overlap protection, stable IDs, SNAP order, trace, rewind, play/pause, resets, stale requests, API errors, desktop and mobile.");
  } catch (error) {
    fs.mkdirSync("test-results", { recursive: true });
    await page.screenshot({ path: "test-results/simulator-failure.png", fullPage: true });
    console.error("Browser errors:", errors, "Page:", await page.locator("body").innerText());
    throw error;
  } finally { await browser.close(); }
})().catch((error) => { console.error(error); process.exitCode = 1; });
