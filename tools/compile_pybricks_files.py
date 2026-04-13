"""
Compile the per-vehicle PyBricks source files into standalone lego_vehicle_timer_*.py files.

Usage (from project root):
    python tools/compile_pybricks_files.py
"""
import os
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent.parent.resolve()
MODULES_DIR  = PROJECT_ROOT / 'modules'


def main() -> None:
    for vehicle in ['servo', 'train', 'skid_steer', 'odv']:
        print(f'Compiling {vehicle}')

        with open(MODULES_DIR / 'lego_vehicle_timer_base.py') as f:
            base_content = f.read()

        with open(MODULES_DIR / f'vehicle_{vehicle}.py') as f:
            import_section_content  = ''
            vars_section_content    = ''
            vehicle_section_content = ''
            drive_section_content   = ''
            import_section  = False
            vars_section    = False
            vehicle_section = False
            drive_section   = False

            for line in f:
                if line.startswith('# IMPORTS_END'):
                    import_section = False
                if import_section:
                    import_section_content += line
                if line.startswith('# IMPORTS_START'):
                    import_section = True

                if line.startswith('# VARS_END'):
                    vars_section = False
                if vars_section:
                    vars_section_content += line
                if line.startswith('# VARS_START'):
                    vars_section = True

                if line.startswith('# MODULE_END'):
                    vehicle_section = False
                if vehicle_section:
                    vehicle_section_content += line
                if line.startswith('# MODULE_START'):
                    vehicle_section = True

                if line.startswith('# DRIVE_SETUP_END'):
                    drive_section = False
                if drive_section:
                    drive_section_content += line
                if line.startswith('# DRIVE_SETUP_START'):
                    drive_section = True

        new_content = base_content.replace('# IMPORT_SECTION',  import_section_content)
        new_content = new_content.replace('# VARS_SECTION',     vars_section_content)
        new_content = new_content.replace('# VEHICLE_SECTION',  vehicle_section_content)
        new_content = new_content.replace('drive_motors = MotorHelper(False, False)', drive_section_content)

        out_path = PROJECT_ROOT / 'docs' / 'pybricks' / f'lego_vehicle_timer_{vehicle}.py'
        if out_path.exists():
            os.remove(out_path)
        with open(out_path, 'w') as f:
            f.write(new_content)


if __name__ == '__main__':
    main()
