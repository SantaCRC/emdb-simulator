import maniskill_ros.robots.ur5e  # imports your robot and registers it
# imports the demo_robot example script and lets you test your new robot
import mani_skill.examples.demo_robot as demo_robot_script


def main() -> None:
    """Run ManiSkill demo with the custom UR5e robot."""

    demo_robot_script.main(
        demo_robot_script.Args(robot_uid="ur5e")
    )  # pass the ur5e robot uid to the demo script to test it


if __name__ == "__main__":
    main()