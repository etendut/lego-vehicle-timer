import os
from pathlib import Path


def main():
    folder_path = Path(__file__).parent.resolve()
    for vehicle in ['servo', 'train','skid_steer','odv']:
        print(f'Compiling {vehicle}')
        with open(Path(folder_path, "lego_vehicle_timer.py")) as base_file:
            base_content = base_file.read()

        with open(Path(folder_path, f"vehicle_{vehicle}.py")) as vehicle_file:
            import_section_content = ""
            vars_section_content = ""
            vehicle_section_content = ""
            drive_section_content = ""
            import_section = False
            vars_section = False
            vehicle_section = False
            drive_section = False
            for line in vehicle_file:
                if line.startswith('#IMPORTS_END'):
                    import_section = False
                if import_section:
                    import_section_content += line
                if line.startswith('#IMPORTS_START'):
                    import_section = True

                if line.startswith('#VARS_END'):
                    vars_section = False
                if vars_section:
                    vars_section_content += line
                if line.startswith('#VARS_START'):
                    vars_section = True

                if line.startswith('#MODULE_END'):
                    vehicle_section = False
                if vehicle_section:
                    vehicle_section_content += line
                if line.startswith('#MODULE_START'):
                    vehicle_section = True

                if line.startswith('#DRIVE_SETUP_END'):
                    drive_section = False
                if drive_section:
                    drive_section_content += line
                if line.startswith('#DRIVE_SETUP_START'):
                    drive_section = True


        new_content = base_content.replace("#IMPORT_SECTION",import_section_content)
        new_content = new_content.replace("#VARS_SECTION",vars_section_content)
        new_content = new_content.replace("#VEHICLE_SECTION",vehicle_section_content)
        new_content = new_content.replace("drive_motors = MotorHelper(False, False)",drive_section_content)

        new_path = Path(folder_path.parent.resolve(),f"lego_vehicle_timer_{vehicle}.py")
        if new_path.exists():
            os.remove(new_path)

        with open(new_path, "w") as new_file:
            new_file.write(new_content)


if __name__ == '__main__':
    main()