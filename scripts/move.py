

def main():
    try:
        mgpi = MoveGroupPythonInteface()
        mgpi.test_run()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    main()