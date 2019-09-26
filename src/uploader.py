#!/usr/bin/env python
import requests
import json
import ast  # convert string represent python expression to python expression


class ontology():
    def __init__(self):
        # self.ontology_server = 'http://192.168.100.16:3030/Brainstorm/'
        self.ontology_server = 'http://localhost:3030/Brainstorm/'
        self.header = {'content-type': 'application/x-www-form-urlencoded'}
        self.prefix = ('prefix kbase: <http://www.semanticweb.org/nico/ontologies/2019/8/Kbase#> '
                       + 'PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#> '
                       + 'PREFIX owl: <http://www.w3.org/2002/07/owl#> '
                       + 'PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> '
                       + 'PREFIX xsd: <http://www.w3.org/2001/XMLSchema#> ')

    def update_property(self, name, property, *argv):
        '''
        This method use to send request to update property value of an
        instance. If there is data parameter, the method gives updating. If
        there is no data parameter, the method gives default value of property.
          param name:     instance's name - string
          param property: property of mentioned instance - string
          param argv:     data of property - string
        '''
        if argv:
            value = argv[0]
        else:
            value = ""

        # Prepair message.
        update = ('DELETE '
                  + 'WHERE { '
                  + 'kbase:{} kbase:{} ?DATA'.format(name, property)
                  + '};'
                  + 'INSERT'
                  + 'DATA { '
                  + 'kbase:{} kbase:{} {}}'.format(name, property, value))

        # send POST request to server
        msg = {'update': self.prefix + update}
        if name != "":
            requests.post(url=self.ontology_server+'update', headers=self.header, data=msg)

    def handle_instance(self, name, type, action):
        '''
        This method use to send request to add or delete an instance.
          param name:    instance's name - string
          param type:    instance's class - string
          param action:  action - INSERT or DELETE
        '''

        # Prepair message.
        update = ('INSERT '
                    + 'DATA { '
                    + 'kbase:{} a owl:NamedIndividual, kbase:{};'.format(name, type)
                    + 'kbase:Current_state  "" ;'
                    + 'kbase:Data           "" ;'
                    + 'kbase:Initial_state  "" ;'
                    + 'kbase:Position       "" ;'
                    + 'kbase:Status         "" .}')

        delete = ('DELETE '
                    + 'WHERE { '
                    + 'kbase:{} ?PROP ?DATA'.format(name)
                    + '}')

        # send POST request to server
        msg = {'update': self.prefix + delete}

        if action == "insert":
            msg = {'update': self.prefix + update}
        if name != "":
            requests.post(url=self.ontology_server+'update', headers=self.header, data=msg)
