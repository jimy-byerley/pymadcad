# author:	Jimy Byerley (jimy.byerley@gmail.com)
# license:	GNU-LGPLv3

""" Intends to replace the well known pprint, to format python objects on output
	
	nprint()      the pprint replacement: it works as print() but works on strings instead of working 
	              on already known objects split too long lines into indented blocks by syntax markers 
	              (such as {}()[], etc)
	              be careful:  as print(), nprint use the string conversion str(obj) to convert an object 
	                           passed, if you want to use the string representation, then you'll have to 
	                           pass repr(obj) to nprint or nformat
	nformat()     split a text into indented lines
	ncolor()      add color makers for terminals syntax highlighting
	deformat()   remove formatting of all marker-enclosed text
"""

from itertools import chain
import os, io


separators = ',;'
begin_marker = '{(['
end_marker = '})]'
symbols = ':*/+-<>'
specials = '\\'
spaces = ' \t\n'

line_width = 100

# blue-green colors
color_encloser =	'\x1b[38;5;42m'
color_marker =		'\x1b[38;5;46m'
color_specials =	'\x1b[38;5;33m'
color_number =		'\x1b[38;5;33m'
color_camel =		'\x1b[1;34;40m'

# red-orange colors
#color_encloser =	'\x1b[38;5;196m'
#color_marker =		'\x1b[38;5;214m'
#color_specials =	'\x1b[38;5;202m'
#color_number =		'\x1b[38;5;202m'
#color_camel =		'\x1b[1;34;40m'

# enable automatic coloration if supported
enable_color = 'COLORTERM' in os.environ


def _next_separator(text, seek):
	counter = 0
	i = seek
	for i in range(seek, len(text)):
		if text[i] in begin_marker:	counter += 1
		elif text[i] in end_marker:	counter -= 1
		if counter < 0 or (counter == 0 and text[i] in separators):	break
	return i

def _next_terminator(text, seek):
	counter = 0
	i = seek
	for i in range(seek, len(text)):
		if text[i] in begin_marker:	counter += 1
		elif text[i] in end_marker:	counter -= 1
		if counter < 0:		break
	return i
	
def _next_word(text, seek):
	while seek < len(text) and text[seek] in spaces:
		seek += 1
	return seek


def nformat(text, indent=0, width=None):
	''' output text, splited into indented blocks, newlines are kept '''
	if type(text) != str:	text = str(text)
	out = io.StringIO()
	seek = 0
	if not width:	width = line_width
	
	while seek < len(text):
		if text[seek] in begin_marker:
			term = _next_terminator(text, seek+1)
			if term - seek > width or '\n' in text[seek:term]:	# develop enclosement if its length is over that width
				indent += 1
				out.write(text[seek])
				if _next_separator(text, seek+1) < term: 	# no return if there is only one element in the enclosement
					out.write( '\n'+'\t'*indent )
					seek = _next_word(text, seek+1)-1
			else:
				out.write(text[seek:term+1])
				seek = term
		elif text[seek] in separators:
			out.write(text[seek] + '\n'+'\t'*indent)
			seek = _next_word(text, seek+1)-1
		elif text[seek] in end_marker:
			indent -= 1
			out.write(text[seek])
		else:
			out.write(text[seek])
		
		seek += 1
	
	return out.getvalue()
	
def deformat(text):
	''' remove formatting from text, leving only indentation and line breaks for lines not ended by a continuation character (,-+*/) '''
	seek = _next_word(text, 0)
	out = io.StringIO()
	
	while seek < len(text):
		if text[seek] in spaces:
			jump = _next_word(text, seek)
			if text[seek-1] in ',+-*/':
				out.write(' ')
			elif text[seek-1] in begin_marker or jump < len(text) and text[jump] in end_marker:
				pass
			elif '\n' in text[seek:jump]:
				out.write('\n')
			else:
				out.write(' ')
			seek = jump
		else:
			out.write(text[seek])
			seek += 1
	return out.getvalue()
		

def _bisect(l, index, key=lambda x:x):
	start,end = 0, len(l)
	while start < end:
		mid = (start+end)//2
		val = key(l[mid])
		if val < index:		start =	mid+1
		elif val > index:	end =	mid
		else:	return mid
	return start

def ncolor(text):
	''' generates a string with the content of the text passed, with terminal color makers to highlight some syntax elements '''
	# list of colors to apply
	coloration = []
	# look for special symbols
	for symbol in chain(separators, begin_marker, end_marker, symbols, specials):
		start = text.find(symbol, 0)
		while start != -1:
			end = start + len(symbol)
			color = color_encloser
			
			if symbol == '<':
				after_name = text.find(' ', end)
				if text[end:after_name].isidentifier():	
					end = after_name
				color = color_marker
			elif symbol == '>':			color = color_marker
			elif symbol in specials:	color = color_specials
			
			coloration.insert(
				_bisect(coloration, start, lambda x:x[1]), 
				(color, start, end-1)
				)
			start = text.find(symbol, end)
	
	# look for special worlds
	it = iter(text)
	i = 0
	for c in it:
		if c.isalnum():
			# look for numbers
			if c.isdigit():
				for j,c in enumerate(it):
					if not c.isalnum():	
						j -= 1
						break
				j += 1
				coloration.insert(
					_bisect(coloration, i, lambda x:x[1]), 
					(color_number, i, i+j)
					)
				i += j+1
			# look for CamelCase names
			#if c.isupper():
				#for j,c in enumerate(it):
					#if not c.isalnum():	
						#j -= 1
						#break
				#j += 1
				#coloration.insert(
					#_bisect(coloration, i, lambda x:x[1]), 
					#(color_camel, i, i+j)
					#)
				#i += j+1
		i += 1
	
	out = ''
	last = 0
	for color,start,end in coloration:
		end +=1
		out += text[last:start] + color + text[start:end] + '\x1b[0m'
		last = end
	out += text[last:]
	return out


def nprint(*args, indent=0, color=enable_color, end='\n'):
	""" write the arguments passed to the standard output, using nformat and ncolor """
	text = '\t'*indent + nformat(' '.join((str(arg) for arg in args)), indent)
	print(ncolor(text) if color else text, end=end)



if __name__ == '__main__':
	# for object dumping with string representation
	nprint(repr(dir()))
	# for common print use
	nprint('here is a list:', [[1000,2324,30],[40000,5000,6342342]], '\nhere is a type:', int, '\nhere is a Name in CamelCase and one in AVeryLongStringAndSoWhat')
	# for string and data output
	nprint('hello everyone, 100 is a decimal number and (0x27a) is an hexadecimal one between parentheses. (did you noticed the automatic line shift ?)')
	# numpy usage with repr() and without
	#import numpy as np
	#m = np.matrix([[1000,2324,30],[40000,5000,6342342]])
	#nprint(repr(m))
	#nprint(m)
	print(deformat('''
	P2 = vec3(0.08182,1.184e-08,-0.09931)
	P3 = vec3(-0.0431,1.258e-08,-0.1056)
P4 = vec3(-0.1593,-1.199e-08,0.1006)
line = [
	Segment(P4, P0),
	Segment(P0, P1 ),
	ArcThrough(P1, vec3(0.09681,-1.713e-09,0.01437), P2
		),
	Segment(P2,P3),
	ArcThrough(P3,vec3(-0.06933,-1.117e-09,0.009369),P4),
	]
axis = Axis(
	vec3(-0.1952,	4.918e-08,	-0.4126),
	vec3(1,	0,	0))
	'''))

	# hardcore test
	#import ast
	#nprint(ast.dump(ast.parse(open(__file__).read())))
