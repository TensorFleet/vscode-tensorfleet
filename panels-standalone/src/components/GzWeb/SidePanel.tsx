import React, { useMemo, useState } from "react";

type SidePanelProps = {
  side?: "left" | "right";
  width?: number;
  buttonSize?: number;
  children?: React.ReactNode;
};

export default function SidePanel({
  side = "right",
  width = 320,
  buttonSize = 32,
  children,
}: SidePanelProps) {
  const [open, setOpen] = useState(true);

  const style = useMemo<React.CSSProperties>(
    () => ({
      ["--sp-width" as any]: `${width}px`,
      ["--sp-btn" as any]: `${buttonSize}px`,
    }),
    [width, buttonSize]
  );

  return (
    <div className={`sp-anchor ${side}`} style={style}>
      <div className={`sp-clip ${open ? "open" : "closed"}`}>
        <div className="sp-panel">
          <div className="sp-content">{children}</div>
        </div>
      </div>

      <button
        type="button"
        className={`sp-toggle ${open ? "open" : "closed"}`}
        onClick={() => setOpen((v) => !v)}
        aria-expanded={open}
      >
        {side === "right" ? (open ? "›" : "‹") : open ? "‹" : "›"}
      </button>
    </div>
  );
}
