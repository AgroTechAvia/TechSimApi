"""Command line interface available from every pip installation."""
from __future__ import annotations
import argparse
import json
import sys
from pathlib import Path

from .calibration.store import DEFAULT_PRESET


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog='python -m agrotechsimapi')
    p.add_argument('--data-dir', type=Path, help='override the writable data directory')
    sub = p.add_subparsers(dest='command', required=True)
    tune = sub.add_parser('calibrate', help='name and run a calibration; preview unless --fly')
    tune.add_argument('--name', required=True)
    tune.add_argument('--base', default=DEFAULT_PRESET, help='profile supplying upstream controllers and initial gains')
    tune.add_argument('--stage', choices=('all','height','yaw','acceleration','velocity','position'), default='all')
    tune.add_argument('--config', type=Path)
    tune.add_argument('--fly', action='store_true')
    tune.add_argument('--replace', action='store_true', help='replace an existing named profile on completion')
    tune.add_argument('--height-stage2', action='store_true', help='skip ascent-only tuning in the height stage')
    manage = sub.add_parser('calibrations', help='list, inspect, export and import profiles')
    actions = manage.add_subparsers(dest='action', required=True)
    actions.add_parser('list')
    show = actions.add_parser('show')
    show.add_argument('name', nargs='?', default=DEFAULT_PRESET)
    init = actions.add_parser('init-config', help='write editable calibration settings')
    init.add_argument('--output', type=Path, required=True)
    export = actions.add_parser('export')
    export.add_argument('name')
    export.add_argument('--output', type=Path, required=True)
    export.add_argument('--replace', action='store_true')
    load = actions.add_parser('import')
    load.add_argument('file', type=Path)
    load.add_argument('--name')
    load.add_argument('--replace', action='store_true')
    report = sub.add_parser('plot', help='generate a single offline HTML snapshot with five tabs')
    source = report.add_mutually_exclusive_group(required=True)
    source.add_argument('--calibration', help='a completed named calibration')
    source.add_argument('--current', action='store_true', help='latest session snapshot, including an active run')
    source.add_argument('--run', type=Path, help='legacy run directory or summary.json')
    report.add_argument('--output', type=Path)
    report.add_argument('--last', type=int, default=0, help='last N trials per stage (0 = all)')
    report.add_argument('--lang', choices=('ru','en'), default='ru')
    report.add_argument('--open', action='store_true', help='open the generated file in your browser')
    return p


def main(argv: list[str] | None = None) -> int:
    p = parser()
    args = p.parse_args(argv)
    from .calibration.store import CalibrationStore, atomic_json
    store = CalibrationStore(args.data_dir)
    try:
        if args.command == 'calibrate':
            from .calibration.service import calibrate
            path = calibrate(store, args.name, stage=args.stage, base=args.base,
                             config_file=args.config, fly=args.fly, replace=args.replace,
                             continuous_height=args.height_stage2)
            if path:
                print(f'Saved calibration: {path}')
                print(f'Validation status: {store.load(args.name)["status"]}')
            return 0
        if args.command == 'calibrations':
            if args.action == 'list':
                print(f'Storage: {store.root}')
                print(f'{"NAME":24} {"CREATED":34} STATUS')
                for profile in store.list():
                    print(f'{profile["name"]:24} {profile["created_at"]:34} {profile["status"]}')
            elif args.action == 'show':
                print(json.dumps(store.load(args.name), ensure_ascii=False, indent=2))
            elif args.action == 'export':
                print(store.export(args.name, args.output, replace=args.replace))
            elif args.action == 'import':
                print(store.import_file(args.file, name=args.name, replace=args.replace))
            elif args.action == 'init-config':
                if args.output.exists():
                    raise FileExistsError(args.output)
                from .calibration.engine import default_config
                config = default_config()
                config.update(store.load()['config'])
                config['output_dir'] = str(store.root / 'runs')
                atomic_json(args.output, config)
                print(args.output.resolve())
            return 0
        if args.command == 'plot':
            if args.last < 0:
                raise ValueError('--last must be nonnegative')
            from .calibration.reports import write_report
            path = write_report(store, name=args.calibration, current=args.current,
                                run=args.run, output=args.output, last=args.last,
                                language=args.lang)
            print(path)
            if args.open:
                import webbrowser
                webbrowser.open(path.as_uri())
            return 0
    except KeyboardInterrupt:
        print('Interrupted; diagnostics remain in runs/', file=sys.stderr)
        return 130
    except (ValueError, OSError, RuntimeError) as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
