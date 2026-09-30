# Совместимость со старыми командами калибратора

Реализация калибратора и построения графиков находится в установленном пакете
`agrotechsimapi.calibration`. Файлы в этом каталоге являются небольшими
совместимыми точками входа для прежних команд и старых прогонов.

Для новых калибровок используйте CLI пакета:

```bash
python -m agrotechsimapi calibrations init-config --output calibration.json
python -m agrotechsimapi calibrate --name my_drone --config calibration.json --fly
python -m agrotechsimapi plot --current --last 8 --open
```

Актуальная документация:

- [практическая инструкция](../docs/calibration_guide.md);
- [техническое описание](../docs/calibration.md).

Локальные `utils/calibration.json`, `utils/calibration_runs/` и созданные HTML
игнорируются Git и не входят в дистрибутив.
