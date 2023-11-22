import json
from roxanne_rosjava_msgs.msg import ActingGoal, Token


class RoxannePlanParser:

    def __init__(self, plan_file):
        self.plan_file = plan_file
        self.MAX_TIME = 1200
        

    def parse(self):
        
        file = open(self.plan_file)

        data = json.load(file)

        goals = []
        id = 1

        for acting_goal in data['acting_goals']:
            goal = ActingGoal()
            goal.goalId = id

            if not acting_goal.get('goal_tokens') is None:
                goal.goals = self.parse_tokens(acting_goal.get('goal_tokens'))

            if not acting_goal.get('fact_tokens') is None:
                goal.facts = self.parse_tokens(acting_goal.get('fact_tokens'))

            goals.append(goal)
            id = id + 1

        
        return goals


    def parse_tokens(self, data_tokens):
        tokens = []
        id = 1
        for token in data_tokens:
            #try:
                tokens.append(self.parse_token(token, id))
                id = id + 1
            #except:
                #continue

        return tokens
    

    def parse_token(self, data_token, id):
        token = Token()
        
        token.id = id
        token.component = str(data_token['component'])
        token.predicate = str(data_token['predicate'])

        for parameter in data_token.get('parameters', []):
            token.parameters.append(str(parameter))

        token.start = data_token.get('start', [0, self.MAX_TIME])
        token.end = data_token.get('end', [1, self.MAX_TIME])
        token.duration = data_token.get('duration', [1, self.MAX_TIME])

        return token