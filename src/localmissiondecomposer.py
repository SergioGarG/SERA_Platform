
# the class is implemented using a template method pattern.
# each slam algorithm extends the class localdecomposer and provides the logic for performing the decomposition of the local missions

def decomposer_init:
    rospy.init_node('decomposer')
    # Subscribers
    rospy.Subscriber("local_mission", String, LocalMissionCallback)
    # Publishers
    TaskPublisher = rospy.Publisher('task_array', TasksArray, queue_size = 100) #Custom message

def LocalMissionCallback(localmission):
    # string containing the local mission

def SendTasks(tasks):
    # sends the array of tasks to be performed

def decomposer(local_mission)
    # code to parse the current local mission and decompose it an array of tasks


if __name__ == '__main__':
    try:
        decomposer_init();
        decomposer(local_mission)
    except rospy.ROSInterruptException:
        pass
