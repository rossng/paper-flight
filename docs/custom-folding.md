# Folding model and limits

The workbench follows preset recipes. Each recipe describes how to fold a sheet, including which layers move at each step. See the [plane library](plane-library.md) for supported designs, references, and downloads.

## Why the sheet keeps its original coordinates

Two points can occupy the same place after folding without being the same piece of paper. Keeping each point's original sheet coordinates preserves material identity through overlapping folds. This lets the flight mesh retain the sheet's area and mass, and lets a flap move separately from the layers beneath it.

The Stable recipe exercises that distinction: its nose flap folds without moving the underlying panels. This checks the geometry of a selective fold; it does not prove that friction would hold a real paper lock.

## What folding checks establish

The recipe checks cover material area, rigid-panel distances, connected seams, and sheet topology. They reject folds that tear the modeled sheet. The panels have zero thickness during folding, so those checks cannot establish collision-free motion, physical stacking order, or whether a fold can be made by hand.

Flight uses a separate flexible model with contact and aerodynamic forces. See [folded flight](folded-flight.md) and [contact limits](self-contact.md). Overlapping paper must not receive multiple copies of the same exposed airflow, and hidden layers must still contribute their mass.

## Downloads and future editing

[FOLD 1.2](https://github.com/edemaine/fold/blob/main/doc/spec.md) downloads contain the original sheet, folded geometry, and this project's recipe history. Internal edges remain unassigned where mountain/valley angles are unknown. Full layer ordering and general FOLD import are not implemented.

An arbitrary crease editor would need to preserve material identity and layer selection, detect invalid later steps after an edit, and handle cuts as changes to sheet connectivity. Changing the dart's dimensions alone cannot provide those guarantees.
