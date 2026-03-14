"""
Recipe system for DropCam recording plans.

Recipes are JSON files stored in the recordings directory under a recipes/ subfolder.
Each recipe defines capture mode, duration, servo movement, and light settings.
"""

import json
import logging
import os
import uuid

logger = logging.getLogger(__name__)

RECIPES_DIR = "/app/videorecordings/recipes"

RECIPE_SCHEMA_DEFAULTS = {
    "name": "Untitled",
    "mode": "video",
    "still_interval_s": 1.0,
    "duration_minutes": 30,
    "auto_start_delay_minutes": 1,
    "rotation_degrees": 0,
    "servo_start_us": 1500,
    "servo_end_us": 1500,
    "servo_sweep_time_s": 30,
    "servo_pause_points": 0,
    "servo_loiter_time_s": 0,
    "servo_oscillations": 1,
    "servo_fixed": True,
    "light_brightness_pct": 0,
    "light_on": False,
}

DEFAULT_RECIPES = [
    {
        "id": "quick-survey-30",
        "name": "Quick Survey 30min",
        "mode": "video",
        "still_interval_s": 1.0,
        "duration_minutes": 30,
        "auto_start_delay_minutes": 1,
        "rotation_degrees": 0,
        "servo_start_us": 1500,
        "servo_end_us": 1500,
        "servo_sweep_time_s": 30,
        "servo_pause_points": 0,
        "servo_loiter_time_s": 0,
        "servo_oscillations": 1,
        "servo_fixed": True,
        "light_brightness_pct": 80,
        "light_on": True,
    },
    {
        "id": "deep-drop-2hr",
        "name": "Deep Drop 2hr",
        "mode": "video",
        "still_interval_s": 1.0,
        "duration_minutes": 120,
        "auto_start_delay_minutes": 5,
        "rotation_degrees": 0,
        "servo_start_us": 1200,
        "servo_end_us": 1800,
        "servo_sweep_time_s": 60,
        "servo_pause_points": 3,
        "servo_loiter_time_s": 5,
        "servo_oscillations": 0,
        "servo_fixed": False,
        "light_brightness_pct": 100,
        "light_on": True,
    },
    {
        "id": "time-lapse-4hr",
        "name": "Time Lapse 4hr",
        "mode": "stills",
        "still_interval_s": 5.0,
        "duration_minutes": 240,
        "auto_start_delay_minutes": 2,
        "rotation_degrees": 0,
        "servo_start_us": 1500,
        "servo_end_us": 1500,
        "servo_sweep_time_s": 30,
        "servo_pause_points": 0,
        "servo_loiter_time_s": 0,
        "servo_oscillations": 1,
        "servo_fixed": True,
        "light_brightness_pct": 50,
        "light_on": True,
    },
]


def _ensure_dir():
    os.makedirs(RECIPES_DIR, exist_ok=True)


def _recipe_path(recipe_id):
    return os.path.join(RECIPES_DIR, f"{recipe_id}.json")


def validate_recipe(data):
    """Validate and fill defaults for a recipe dict. Returns (clean_dict, errors)."""
    errors = []
    clean = dict(RECIPE_SCHEMA_DEFAULTS)

    if "name" in data:
        clean["name"] = str(data["name"]).strip()[:80]
    if not clean["name"]:
        errors.append("name is required")

    if "mode" in data:
        if data["mode"] not in ("video", "stills"):
            errors.append("mode must be 'video' or 'stills'")
        else:
            clean["mode"] = data["mode"]

    for fld, lo, hi in [
        ("still_interval_s", 0.1, 3600),
        ("duration_minutes", 1, 480),
        ("auto_start_delay_minutes", 0, 60),
        ("servo_start_us", 1000, 2000),
        ("servo_end_us", 1000, 2000),
        ("servo_sweep_time_s", 1, 600),
        ("servo_pause_points", 0, 50),
        ("servo_loiter_time_s", 0, 120),
        ("servo_oscillations", 0, 999),
        ("light_brightness_pct", 0, 100),
    ]:
        if fld in data:
            try:
                val = float(data[fld])
                if val < lo or val > hi:
                    errors.append(f"{fld} must be between {lo} and {hi}")
                else:
                    clean[fld] = int(val) if isinstance(RECIPE_SCHEMA_DEFAULTS[fld], int) else round(val, 1)
            except (ValueError, TypeError):
                errors.append(f"{fld} must be a number")

    if "rotation_degrees" in data:
        rd = int(data["rotation_degrees"])
        if rd not in (0, 90, 180, 270):
            errors.append("rotation_degrees must be 0, 90, 180, or 270")
        else:
            clean["rotation_degrees"] = rd

    for bf in ("servo_fixed", "light_on"):
        if bf in data:
            clean[bf] = bool(data[bf])

    if "id" in data:
        clean["id"] = str(data["id"])

    return clean, errors


def init_default_recipes():
    """Create default recipe files if the recipes dir is empty."""
    _ensure_dir()
    existing = list_recipes()
    if existing:
        return
    for recipe in DEFAULT_RECIPES:
        save_recipe(recipe, recipe["id"])
    logger.info(f"Initialized {len(DEFAULT_RECIPES)} default recipes")


def list_recipes():
    """Return list of all saved recipes (dicts with id + name)."""
    _ensure_dir()
    recipes = []
    for fname in sorted(os.listdir(RECIPES_DIR)):
        if not fname.endswith(".json"):
            continue
        try:
            with open(os.path.join(RECIPES_DIR, fname), "r") as f:
                r = json.load(f)
            r["id"] = fname[:-5]
            recipes.append(r)
        except Exception as e:
            logger.warning(f"Bad recipe file {fname}: {e}")
    return recipes


def get_recipe(recipe_id):
    """Load a single recipe by id. Returns dict or None."""
    path = _recipe_path(recipe_id)
    if not os.path.exists(path):
        return None
    try:
        with open(path, "r") as f:
            r = json.load(f)
        r["id"] = recipe_id
        return r
    except Exception as e:
        logger.error(f"Failed to load recipe {recipe_id}: {e}")
    return None


def save_recipe(data, recipe_id=None):
    """Validate and save a recipe. Returns (recipe_dict, errors)."""
    _ensure_dir()
    clean, errors = validate_recipe(data)
    if errors:
        return None, errors
    if recipe_id is None:
        recipe_id = clean.get("id", uuid.uuid4().hex[:12])
    clean["id"] = recipe_id
    path = _recipe_path(recipe_id)
    try:
        with open(path, "w") as f:
            json.dump(clean, f, indent=2)
        return clean, []
    except Exception as e:
        return None, [str(e)]


def delete_recipe(recipe_id):
    """Delete a recipe file. Returns True on success."""
    path = _recipe_path(recipe_id)
    if os.path.exists(path):
        os.remove(path)
        return True
    return False
