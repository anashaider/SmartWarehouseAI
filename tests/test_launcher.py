"""Launcher tests: 3D is the default and the 2D dashboard is the fallback."""

from __future__ import annotations

import interface3d
import main


def test_falls_back_to_2d_when_panda3d_missing(monkeypatch, capsys):
    monkeypatch.setattr(interface3d, "panda3d_available", lambda: (False, "Panda3D is not installed"))
    args = main.parse_args([])
    config = main.SimulationConfig()
    assert main.launch_3d(config, args) is False
    assert "Falling back to the 2D dashboard" in capsys.readouterr().out


def test_cli_options():
    args = main.parse_args(["--2d", "--quality", "high", "--present", "--no-intro"])
    assert args.two_d and args.quality == "high" and args.present and args.no_intro
    assert main.parse_args([]).two_d is False


def test_window_size_is_sane():
    w, h = main._window_size()
    assert 1280 <= w <= 1920 and 720 <= h <= 1080
