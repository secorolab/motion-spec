// SPDX-License-Identifier: MPL-2.0
// SPDX-FileCopyrightText: 2026 SECORO AG (secoro.uni-bremen.de)
// Author: Vamsi Kalagaturu

let menuId = 0;

const close = (control) => {
  control.shell.classList.remove("open");
  control.button.setAttribute("aria-expanded", "false");
};

const choices = (control) => [...control.menu.querySelectorAll("[role=option]")];

const refresh = (control) => {
  const { select, button, menu } = control;
  button.textContent = select.selectedOptions[0]?.textContent ?? "Select";
  button.disabled = select.disabled;
  menu.replaceChildren(...[...select.options].map((option, index) => {
    const choice = document.createElement("button");
    choice.type = "button";
    choice.className = "theme-select-option";
    choice.textContent = option.textContent;
    choice.disabled = option.disabled;
    choice.id = `${menu.id}-${index}`;
    choice.setAttribute("role", "option");
    choice.setAttribute("aria-selected", String(option.selected));
    choice.onclick = () => {
      select.value = option.value;
      select.dispatchEvent(new Event("change", { bubbles: true }));
      close(control);
      button.focus();
    };
    choice.onkeydown = (event) => {
      const entries = choices(control);
      const current = entries.indexOf(choice);
      if (event.key === "Escape") return (close(control), button.focus());
      if (event.key === "Home" || event.key === "End") {
        event.preventDefault();
        return entries[event.key === "Home" ? 0 : entries.length - 1]?.focus();
      }
      if (event.key === "ArrowDown" || event.key === "ArrowUp") {
        event.preventDefault();
        return entries[(current + (event.key === "ArrowDown" ? 1 : -1) + entries.length) % entries.length]?.focus();
      }
      if (event.key === "Tab") close(control);
    };
    return choice;
  }));
};

export const enhanceSelect = (select) => {
  if (select.multiple || select.dataset.themeSelect) return;
  select.dataset.themeSelect = "true";
  const shell = document.createElement("div");
  shell.className = "theme-select";
  const button = document.createElement("button");
  button.type = "button";
  button.className = "theme-select-trigger";
  button.setAttribute("aria-haspopup", "listbox");
  button.setAttribute("aria-expanded", "false");
  button.setAttribute("aria-label", select.getAttribute("aria-label") ?? "Select");
  const menu = document.createElement("div");
  menu.className = "theme-select-menu";
  menu.id = `select-menu-${menuId++}`;
  menu.setAttribute("role", "listbox");
  button.setAttribute("aria-controls", menu.id);
  select.before(shell);
  shell.append(select, button, menu);
  select.classList.add("theme-select-native");
  select.tabIndex = -1;
  select.setAttribute("aria-hidden", "true");
  select.removeAttribute("aria-label");
  const control = { select, button, menu, shell };
  button.onclick = () => {
    const opening = !shell.classList.contains("open");
    document.querySelectorAll(".theme-select.open").forEach((entry) => close(entry.control));
    if (!opening) return;
    shell.classList.add("open");
    button.setAttribute("aria-expanded", "true");
    choices(control).find((entry) => entry.getAttribute("aria-selected") === "true")?.focus();
  };
  button.onkeydown = (event) => {
    if (event.key === "Escape") return close(control);
    if (["ArrowDown", "ArrowUp", "Enter", " "].includes(event.key)) {
      event.preventDefault();
      button.click();
    }
  };
  shell.control = control;
  select.addEventListener("change", () => refresh(control));
  new MutationObserver(() => refresh(control)).observe(select, { childList: true, subtree: true });
  refresh(control);
};

export const installSelectTheme = () => {
  document.querySelectorAll("select").forEach(enhanceSelect);
  new MutationObserver((changes) => {
    changes.forEach((change) => change.addedNodes.forEach((node) => {
      if (node.nodeType !== Node.ELEMENT_NODE) return;
      if (node.matches("select")) enhanceSelect(node);
      node.querySelectorAll?.("select").forEach(enhanceSelect);
    }));
  }).observe(document.body, { childList: true, subtree: true });
  document.addEventListener("click", (event) => {
    if (!event.target.closest(".theme-select")) document.querySelectorAll(".theme-select.open").forEach((entry) => close(entry.control));
  });
};
