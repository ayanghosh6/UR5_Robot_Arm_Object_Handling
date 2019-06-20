#!/usr/bin/env python
import os
import rospy
import threading
import roslib.rosenv 

from flask import Flask
from flask_ask import Ask, question, statement
from std_msgs.msg import String

app = Flask(__name__)
ask = Ask(app, "/")

# ROS node, publisher, and parameter.
# The node is started in a separate thread to avoid conflicts with Flask.
# The parameter *disable_signals* must be set if node is not initialized
# in the main thread.

threading.Thread(target=lambda: rospy.init_node('test_node', disable_signals=True)).start()
pub = rospy.Publisher('test_pub', String, queue_size=1)
NGROK = rospy.get_param('/ngrok', None)


@ask.launch
def launch():
    '''
    Executed when launching skill: say "Alexa, ask tester"
    '''
    welcome_sentence = 'Hello. Please state a command for the robot.'
    return question(welcome_sentence)


@ask.intent('TestIntent')
def test_intent_function():
    '''
    Executed when "TestIntent" is called:
    say "Alexa, ask tester to say (first name of a person)"
    Note that the 'intent_name' argument of the decorator @ask.intent
    must match the name of the intent in the Alexa skill.
    '''
    pub.publish('pickup')
    return statement('The robot is picking up the object')

@ask.intent('TestDropIntent')
def test_drop_intent_function():
    '''
    Executed when "TestIntent" is called:
    say "Alexa, ask tester to say (first name of a person)"
    Note that the 'intent_name' argument of the decorator @ask.intent
    must match the name of the intent in the Alexa skill.
    '''
    pub.publish('drop')
    return statement('The robot is dropping the object')



@ask.session_ended
def session_ended():
    return "{}", 200


if __name__ == '__main__':
    if NGROK:
        print 'NGROK mode'
        print (os.environ['HOME'])
        app.run(host='localhost', port=5000)
        #app.run(debug=true)

    else:
        print 'Manual tunneling mode'
        dirpath = os.path.dirname(__file__)
        cert_file = os.path.join(dirpath, '../config/ssl_keys/certificate.pem')
        pkey_file = os.path.join(dirpath, '../config/ssl_keys/private-key.pem')
        app.run(host=os.environ['ROS_IP'], port=5000,
                ssl_context=(cert_file, pkey_file))
    
