
class PlpAchieveResultFailureScenario:
    """
    A failure scenario, complete with its probability, estimated time to fail, and name.
    """
    def __init__(self, name, probability, time):
        self.name = name
        self.probability = probability
        self.time = time
        
    def __repr__(self):
        return "(%s p:%s t:%s)" % (self.name, self.probability, self.time)


class PlpAchieveResult:
    """
    A result of an estimation of a PLP achieve module.
    Fields are:
    success: probability ([0..1])
    success_time: Time to achieve success
    confidence: How sure are we about the success estimation
    side_effects: Dictionary from string to the side effect (e.g. {'fuel':-18})
    failure: Dictionary of string to PlpAchieveResultFailScenario.
    """
    def __init__(self):
        self.success = None
        self.success_time = None
        self.confidence = None
        self.side_effects = {}
        self.failure = {}

    def add_failure(self, fail):
        self.failure[fail.name] = fail
        
    def __repr__(self):
        return "PlpAchieveResult( success:%s success_time:%s confidence:%s side_effects:%s failure:%s )" % \
                    (self.success, self.success_time, self.confidence, repr(self.side_effects), repr(self.failure))
                    
                