#!/usr/bin/env python
import requests
import json


class ontology():
    def __init__(self):
        self.detected_objects = []
        self.ontology_server = 'http://localhost:3030/Testing/'
        self.header = {'content-type': 'application/x-www-form-urlencoded'}
        self.prefix = ('PREFIX brainstorm:<http://www.semanticweb.org/led/ontologies/2019/4/brainstorm.owl#> '
                       + 'PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#> '
                       + 'PREFIX owl: <http://www.w3.org/2002/07/owl#> '
                       + 'PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> '
                       + 'PREFIX xsd: <http://www.w3.org/2001/XMLSchema#> ')

    def update_ontology(self, object, type):
        # type is DELETE or INSERT
        name, raw_string, corners = object.split("/")
        location = raw_string.split(" ")
        box = corners.split(" ")
        if name not in self.detected_objects:
            self.detected_objects.append(name)
            # Prepair message
            insert = (' Data { '
                      + 'brainstorm:{}_status a owl:NamedIndividual , '.format(name)
                      + 'brainstorm:informations ; '
                      + 'brainstorm:xmin {}; '.format(box[1])
                      + 'brainstorm:xmax {}; '.format(box[3])
                      + 'brainstorm:ymin {}; '.format(box[0])
                      + 'brainstorm:ymax {}. '.format(box[2])

                      + 'brainstorm:{}_location a owl:NamedIndividual , '.format(name)
                      + 'brainstorm:informations ; '
                      + 'brainstorm:x {}; '.format(location[0])
                      + 'brainstorm:y {}; '.format(location[1])
                      + 'brainstorm:z {}. '.format(location[2])

                      + 'brainstorm:{} a owl:NamedIndividual , brainstorm:objects ;'.format(name)
                      + ' brainstorm:hasLocation brainstorm:{}_location; '.format(name)
                      + 'brainstorm:hasStatus brainstorm:{}_status.'.format(name)
                      + '}'
                      )
            # send POST request to server
            msg = {'update': self.prefix + type + insert}
            requests.post(url=self.ontology_server+'update', headers=self.header, data=msg)
        return True

    def get_info(self, name):
        query = ("SELECT ?x ?y ?z ?xmin ?ymin ?xmax ?ymax " +
                 "WHERE { " +
                 "brainstorm:{}_location brainstorm:x ?x. ".format(name) +
                 "brainstorm:{}_location brainstorm:y ?y. ".format(name) +
                 "brainstorm:{}_location brainstorm:z ?z. ".format(name) +
                 "brainstorm:{}_status brainstorm:xmin ?xmin. ".format(name) +
                 "brainstorm:{}_status brainstorm:ymin ?ymin. ".format(name) +
                 "brainstorm:{}_status brainstorm:xmax ?xmax. ".format(name) +
                 "brainstorm:{}_status brainstorm:ymax ?ymax. ".format(name) +
                 "}")
        # send POST request to server
        msg = {'query': self.prefix + query}
        r = requests.post(url=self.ontology_server+'query', headers=self.header, data=msg)
        # Handle response as json type
        res = json.loads(r.content)
        location = [
            float(res["results"]["bindings"][0]["x"]["value"]),
            float(res["results"]["bindings"][0]["y"]["value"]),
            float(res["results"]["bindings"][0]["z"]["value"])]
        status = [
            float(res["results"]["bindings"][0]["xmin"]["value"]),
            float(res["results"]["bindings"][0]["ymin"]["value"]),
            float(res["results"]["bindings"][0]["xmax"]["value"]),
            float(res["results"]["bindings"][0]["ymax"]["value"])]
        return {name: {"location": location, "status": status}}

    def get_name(self):
        objects = []
        query = ("SELECT ?name " +
                 "WHERE {?name a  owl:NamedIndividual , brainstorm:objects .}")
        msg = {'query': self.prefix + query}
        r = requests.post(url=self.ontology_server+'query', headers=self.header, data=msg)
        res = json.loads(r.content)
        for i in res["results"]["bindings"]:
            _, name = i["name"]["value"].split("#")
            objects.append(name)
        return objects

    def get_relative_position(self, name):
        left = []
        right = []
        up = []
        down = []
        ref_object = self.get_info(name)[name]['status']
        for object in self.detected_objects:
            if name == object:
                continue

            target_object = self.get_info(object)[object]['status']

            if target_object[0] > ref_object[2]:
                right.append(object)
            if target_object[2] < ref_object[0]:
                left.append(object)
            if target_object[1] > ref_object[3]:
                up.append(object)
            if target_object[3] < ref_object[1]:
                down.append(object)

        return {"LEFT": left, "RIGHT": right, "UP": up, "DOWN": down}
