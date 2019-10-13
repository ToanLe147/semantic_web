#!/usr/bin/env python
import requests
import json
import ast  # convert string represent python expression to python expression


class Ontology():
    def __init__(self):
        # self.ontology_server = 'http://192.168.100.16:3030/Brainstorm/'
        self.ontology_server = 'http://localhost:3030/Brainstorm/'
        self.header = {'content-type': 'application/x-www-form-urlencoded'}
        self.prefix = ('PREFIX kbase: <http://www.semanticweb.org/nico/ontologies/2019/8/Kbase#> '
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
            value = '"' + '{}'.format(argv[0]) + '"'
        else:
            value = ""

        # Prepair message.
        update = ('DELETE '
                  + 'WHERE {'
                  + 'kbase:{} kbase:{} ?DATA'.format(name, property)
                  + '};'
                  + 'INSERT'
                  + 'DATA {'
                  + 'kbase:{} kbase:{} {}'.format(name, property, value)
                  + '}')

        # send POST request to server
        msg = {'update': self.prefix + update}
        r = requests.post(url=self.ontology_server+'update', headers=self.header, data=msg)
        if r.status_code == 200:
            print("Property updated")
            return "Property updated"

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
        res = "Instance deleted"
        if action == "insert":
            msg = {'update': self.prefix + update}
            res = "Instance updated"
        if name != "":
            r = requests.post(url=self.ontology_server+'update', headers=self.header, data=msg)
            if r.status_code == 200:
                return res

    def get_instances(self):
        '''
        This method gets list of data in Knowledge base.
        '''
        # Prepair message.
        content = ('SELECT * '
                    + 'WHERE { '
                    + '?NAME a owl:NamedIndividual}')
        msg = {'query': self.prefix + content}
        r = requests.post(url=self.ontology_server+'query', headers=self.header, data=msg)
        # Handle response as json type
        res = json.loads(r.content)
        # print(self.instance_list)
        update_list = []
        for i in res["results"]["bindings"]:
            name = i["NAME"]["value"].split("#")[1]
            update_list.append(name)
        return update_list

    def get_relationship(self):
        '''
        This method gets list of object properties in Knowledge base.
        '''
        # Prepair message.
        content = ('SELECT * '
                    + 'WHERE { '
                    + '?NAME a owl:ObjectProperty}')
        msg = {'query': self.prefix + content}
        r = requests.post(url=self.ontology_server+'query', headers=self.header, data=msg)
        # Handle response as json type
        res = json.loads(r.content)
        update_list = []
        for i in res["results"]["bindings"]:
            name = i["NAME"]["value"].split("#")[1]
            update_list.append(name)
        return update_list

    def handle_relationship(self, relationship, *argv):
        '''
        This method updates the relationships between object instances.
        '''
        if argv:
            name1 = argv[0]
            name2 = argv[1]
            # Prepair message
            content = ('INSERT '
                        + 'DATA { '
                        + 'kbase:{} kbase:{} kbase:{}'.format(name1, relationship, name2)
                        + '}')
        else:
            # Prepair message
            content = ('INSERT '
                        + 'DATA { '
                        + 'kbase:{} a owl:ObjectProperty'.format(relationship)
                        + '}')
        msg = {'update': self.prefix + content}
        r = requests.post(url=self.ontology_server+'update', headers=self.header, data=msg)
        if r.status_code == 200:
            return "Updated relationship"
