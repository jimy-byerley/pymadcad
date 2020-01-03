# author:	Jimy Byerley (jimy.byerley@gmail.com)

""" Intends to replace the well known pprint, to format python objects on output
	
	nprint()      the pprint replacement: it works as print() but works on strings instead of working 
	              on already known objects split too long lines into indented blocks by syntax markers 
	              (such as {}()[], etc)
	              be careful:  as print(), nprint use the string conversion str(obj) to convert an object 
	                           passed, if you want to use the string representation, then you'll have to 
	                           pass repr(obj) to nprint or nformat
	nformat()     split a text into indented lines
	ncolor()      add color makers for terminals syntax highlighting
"""

from itertools import chain
import os


separators = {',', ';'}
begin_marker = {'{', '(', '['}
end_marker = {'}', ')', ']'}
symbols = {':', '*', '/', '+', '-', '<', '>'}
specials = {'\\'}

line_length = 100

# blue-green colors
#color_encloser =	'\x1b[0;36;40m'
#color_marker =		'\x1b[0;37;40m'
#color_specials =	'\x1b[1;31;40m'
#color_number =		'\x1b[1;34;40m'

# red-orange colors
color_encloser =	'\x1b[0;31;40m'
color_marker =		'\x1b[0;37;40m'
color_specials =	'\x1b[1;93;40m'
color_number =		'\x1b[1;91;40m'
color_camel =		'\x1b[1;34;40m'

# enable automatic coloration if supported
enable_color = 'COLORTERM' in os.environ


def _next_separator(text, seek):
	counter = 0
	for i in range(seek, len(text)):
		if text[i] in begin_marker:	counter += 1
		elif text[i] in end_marker:	counter -= 1
		if counter < 0 or (counter == 0 and text[i] in separators):	break
	return i

def _next_terminator(text, seek):
	counter = 0
	for i in range(seek, len(text)):
		if text[i] in begin_marker:	counter += 1
		elif text[i] in end_marker:	counter -= 1
		if counter < 0:		break
	return i
'''
def next_beginner(text, seek):
	for i in range(seek, len(text)):
		if text[i] in begin_marker:	break
	return i
'''
	
def nformat(text, indent=0):
	""" output text, splited into indented blocks, newlines are kept """
	if type(text) != str:	text = str(text)
	out = ""
	seek = 0
	
	while seek < len(text):
		if text[seek] in begin_marker:
			term = _next_terminator(text, seek+1)
			if term - seek > line_length or '\n' in text[seek:term]:	# develop enclosement if its length is over that line_length
				indent += 1
				out += text[seek]
				if _next_separator(text, seek+1) < term: 	# no return if there is only one element in the enclosement
					out += '\n'+'\t'*indent
			else:
				out += text[seek:term+1]
				seek = term
		elif text[seek] in separators:
			out += text[seek] + '\n'+'\t'*indent
		elif text[seek] in end_marker:
			indent -= 1
			out += text[seek]
		elif out and out[-1] in {'\t', '\n'} and text[seek] in {' ', '\t', '\n'}:	# remove spaced near ligne switch
		#elif text[seek] in {' ', '\t', '\n'}:
			pass
		else:
			out += text[seek]
		
		seek += 1
	
	return out

def _dichotomy_index(l, index, key=lambda x:x):
	start,end = 0, len(l)
	while start < end:
		mid = (start+end)//2
		val = key(l[mid])
		if val < index:		start =	mid+1
		elif val > index:	end =	mid
		else:	return mid
	return start

def ncolor(text):
	""" generates a string with the content of the text passed, with terminal color makers to highlight some syntax elements """
	# liste des colorations a appliquer
	coloration = []
	# recherche des symboles spéciaux
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
				_dichotomy_index(coloration, start, lambda x:x[1]), 
				(color, start, end-1)
				)
			start = text.find(symbol, end)
	
	# recherche des noms CamelCase
	it = iter(text)
	i = 0
	for c in it:
		if c.isalnum():
			# recherce des nombres (base quelconque)
			if c.isdigit():
				for j,c in enumerate(it):
					if not c.isalnum():	
						j -= 1
						break
				j += 1
				coloration.insert(
					_dichotomy_index(coloration, i, lambda x:x[1]), 
					(color_number, i, i+j)
					)
				i += j+1
			# recherche des noms CamelCase
			#if c.isupper():
				#for j,c in enumerate(it):
					#if not c.isalnum():	
						#j -= 1
						#break
				#j += 1
				#coloration.insert(
					#_dichotomy_index(coloration, i, lambda x:x[1]), 
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
