% ----------------------------------------------------------------------------
% FIT thesis
% ----------------------------------------------------------------------------
% (C) 2007 Zdenek Vasicek, 2008 Michal Bidlo
% ----------------------------------------------------------------------------
% Šablonu upravil: Ing. Jaroslav Dytrych, idytrych@fit.vutbr.cz
% ----------------------------------------------------------------------------
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{fitthesis}[2007/04/13 Sablona pro BP/DP]

%Vychozi hodnoty parametru
\newif\ifczech \let\ifczech\iftrue
\newif\ifslovak \let\ifslovak\iffalse
\newif\ifJVS \let\ifJVS\iffalse
\newif\ifVAFLE \let\ifVAFLE\iffalse
\newif\ifOPEN \let\ifOPEN\iffalse
\newif\ifARIAL \let\ifARIAL\iffalse
\newif\ifTVAFLE \let\ifTVAFLE\iffalse
\newif\ifTOPEN \let\ifTOPEN\iffalse
\newif\ifTBOPEN \let\ifTBOPEN\iffalse
\newif\ifTARIAL \let\ifTARIAL\iffalse
\newif\ifGRAY \let\ifGRAY\iffalse
\newif\ifTIMES \let\ifTIMES\iffalse

\newif\ifPrint \let\ifPrint\iffalse
\newif\ifWis   \let\ifWis\iftrue
\newif\ifzadani   \let\ifzadani\iffalse


%-----------------------------------------------------------------------------
%Parametry tridy (odkomentovani zakomentovanych na vlastni nebezpeci)
%-----------------------------------------------------------------------------
\DeclareOption{english}{\let\ifczech\iffalse}
\DeclareOption{slovak}{\let\ifslovak\iftrue\let\ifczech\iffalse}
\DeclareOption{print}{\let\ifWis\iffalse \let\ifPrint\iftrue}
\DeclareOption{zadani}{\let\ifzadani\iftrue}
%\DeclareOption{JVS}{\let\ifJVS\iftrue}
%\DeclareOption{VAFLE}{\let\ifVAFLE\iftrue}
%\DeclareOption{OPEN}{\let\ifOPEN\iftrue}
%\DeclareOption{ARIAL}{\let\ifARIAL\iftrue}
%\DeclareOption{TVAFLE}{\let\ifTVAFLE\iftrue}
%\DeclareOption{TOPEN}{\let\ifTOPEN\iftrue}
%\DeclareOption{TBOPEN}{\let\ifTBOPEN\iftrue}
%\DeclareOption{TARIAL}{\let\ifTARIAL\iftrue}
%\DeclareOption{GRAY}{\let\ifGRAY\iftrue}
%\DeclareOption{TIMES}{\let\ifTIMES\iftrue}

\DeclareOption*{}
\ProcessOptions\relax
%Jako vychozi pouzijeme tridu REPORT
\ifx\pdfoutput\undefined % nejedeme pod pdftexem
   \LoadClass[a4paper,oneside,onecolumn,11pt]{report}
\else
   \LoadClass[pdftex,a4paper,oneside,onecolumn,11pt]{report}
\fi

%-----------------------------------------------------------------------------
%Balicky
%-----------------------------------------------------------------------------
\usepackage{amsmath}
\usepackage{amssymb}
\usepackage[czech,slovak,english]{babel}
\usepackage[utf8]{inputenc} %kodovani
\usepackage[T1]{fontenc}
\usepackage{cmap}
\usepackage{url}
\usepackage{ragged2e}
\usepackage{color}
\usepackage{etoolbox}
\ifczech
  \main@language{czech}
\else
  \ifslovak
    \main@language{slovak}
  \else
    \main@language{english}
  \fi
\fi
\ifx\pdfoutput\undefined  % nejedeme pod pdftexem
  \usepackage{graphics}
  \usepackage{epsfig}
\else % je to pdftex !
  \ifzadani
    \usepackage{pdfpages}
  \else % neni potreba pdfpages
    \usepackage[pdftex]{graphicx}
  \fi
\fi

\definecolor{VUTGRAY}{RGB}{138,140,144}

\newtoggle{slovak_used}
\togglefalse{slovak_used}

\ifslovak
  \toggletrue{slovak_used}
\fi

\newtoggle{declare_open}
\togglefalse{declare_open}

\ifVAFLE
  \toggletrue{declare_open}
\fi
\ifTVAFLE
  \toggletrue{declare_open}
\fi
\ifTOPEN
  \toggletrue{declare_open}
\fi
\ifTBOPEN
  \toggletrue{declare_open}
\fi

%-----------------------------------------------------------------------------
%Sazebni obrazec
%-----------------------------------------------------------------------------
\setlength{\hoffset}{-0.7cm}
\setlength{\voffset}{-1.5cm}
\setlength{\textheight}{23.0cm}
\setlength{\textwidth}{15.2cm}

%-----------------------------------------------------------------------------
%Texty
%-----------------------------------------------------------------------------
\def\@vut@CS{Vysoké učení technické v~Brně}
\def\@vut@SK{Vysoké učení technické v~Brně}
\def\@vut@EN{Brno University of Technology}
\def\@fit@CS{Fakulta informačních technologií}
\def\@fit@SK{Fakulta informačních technologií}
\def\@fit@EN{Faculty of Information Technology}
\def\@author@CS{Autor práce}
\def\@author@SK{Autor práce}
\def\@author@EN{Author}
\def\@dep@CS{Ústav}
\def\@dep@SK{Ústav}
\def\@dep@EN{Department of}
\def\@abstract@CS{Abstrakt}
\def\@abstract@SK{Abstrakt}
\def\@abstract@EN{Abstract}
\def\@keywords@CS{Klíčová slova}
\def\@keywords@SK{Kľúčové slová}
\def\@keywords@EN{Keywords}
\def\@cite@CS{Citace}
\def\@cite@SK{Citácia}
\def\@cite@EN{Reference}
\def\@copyright@CS{Tato práce vznikla jako školní dílo na Vysokém učení technickém v Brně, Fakultě informačních technologií. 
Práce je chráněna autorským zákonem a její užití bez udělení oprávnění autorem je nezákonné, s výjimkou zákonem definovaných případů.}
\def\@copyright@SK{Táto práca vznikla ako školské dielo na FIT VUT v Brně. 
Práca je chránená autorským zákonom a jej využitie bez poskytnutia oprávnenia autorom je nezákonné, s výnimkou zákonne definovaných prípadov.}
\def\@copyright@EN{This thesis was created as a school work at the Brno University of Technology, Faculty of Information Technology.
The thesis is protected by copyright law and its use without author's explicit consent is illegal, except for cases defined by law.}
\def\@tdecl@CS{Prohlášení}
\def\@tdecl@SK{Prehlásenie}
\def\@tdecl@EN{Declaration}
\def\@tackn@CS{Poděkování}
\def\@tackn@SK{Poďakovanie}
\def\@tackn@EN{Acknowledgements}

\newif\ifnasel
\def\ifEqString#1#2#3#4{\def\testa{#1}\def\testb{#2}%
  \ifx\testa\testb \def#4{#3} \naseltrue \fi%
}

%Vyber typu projektu
\newcommand{\set@project}[1]{%
 \naselfalse
 \ifEqString{#1}{BP}{Bakalářská práce}{\@projecttype@CS}%
% \ifEqString{#1}{BP}{Bakalárska práca}{\@projecttype@SK}%
 \ifEqString{#1}{BP}{Bakalářská práce}{\@projecttype@SK}%
 \ifEqString{#1}{BP}{Bachelor's thesis}{\@projecttype@EN}%
 \ifEqString{#1}{BP}{Vedoucí práce}{\@supervisor@CS}%
% \ifEqString{#1}{BP}{Vedúci práce}{\@supervisor@SK}%
 \ifEqString{#1}{BP}{Vedoucí práce}{\@supervisor@SK}%
 \ifEqString{#1}{BP}{Supervisor}{\@supervisor@EN}%
 \ifEqString{#1}{SP}{Semestrální projekt}{\@projecttype@CS}%
% \ifEqString{#1}{SP}{Semestrálny projekt}{\@projecttype@SK}%
 \ifEqString{#1}{SP}{Semestrální projekt}{\@projecttype@SK}%
 \ifEqString{#1}{SP}{Term project}{\@projecttype@EN}%
 \ifEqString{#1}{SP}{Vedoucí práce}{\@supervisor@CS}%
% \ifEqString{#1}{SP}{Vedúci práce}{\@supervisor@SK}%
 \ifEqString{#1}{BP}{Vedoucí práce}{\@supervisor@SK}%
 \ifEqString{#1}{SP}{Supervisor}{\@supervisor@EN}%
 \ifEqString{#1}{DP}{Diplomová práce}{\@projecttype@CS}%
% \ifEqString{#1}{DP}{Diplomová práca}{\@projecttype@SK}%
 \ifEqString{#1}{DP}{Diplomová práce}{\@projecttype@SK}%
 \ifEqString{#1}{DP}{Master's thesis}{\@projecttype@EN}%
 \ifEqString{#1}{DP}{Vedoucí práce}{\@supervisor@CS}%
% \ifEqString{#1}{DP}{Vedúci práce}{\@supervisor@SK}%
 \ifEqString{#1}{DP}{Vedoucí práce}{\@supervisor@SK}%
 \ifEqString{#1}{DP}{Supervisor}{\@supervisor@EN}%
 \ifEqString{#1}{DR}{Disertační práce}{\@projecttype@CS}%
% \ifEqString{#1}{DR}{Dizertačná práca}{\@projecttype@SK}%
 \ifEqString{#1}{DR}{Disertační práce}{\@projecttype@SK}%
 \ifEqString{#1}{DR}{PhD thesis}{\@projecttype@EN}%
 \ifEqString{#1}{DR}{Školitel}{\@supervisor@CS}%
% \ifEqString{#1}{DR}{Školiteľ}{\@supervisor@SK}%
 \ifEqString{#1}{DR}{Školitel}{\@supervisor@SK}%
 \ifEqString{#1}{DR}{Supervisor}{\@supervisor@EN}%
 \ifnasel
 \else
   \typeout{Spatny typ projektu! Moznosti: BP, SP, DP, DR}\set@project{BP}
 \fi
}

%Vyber ustavu
\newcommand{\set@department}[1]{%
 \naselfalse
 \ifEqString{#1}{UIFS}{informačních systémů}{\@department@CS}%
 \ifEqString{#1}{UIFS}{informačních systémů}{\@department@SK}%
 \ifEqString{#1}{UIFS}{Information Systems}{\@department@EN}%
 \ifEqString{#1}{UPSY}{počítačových systémů}{\@department@CS}%
 \ifEqString{#1}{UPSY}{počítačových systémů}{\@department@SK}%
 \ifEqString{#1}{UPSY}{Computer Systems}{\@department@EN}%
 \ifEqString{#1}{UITS}{inteligentních systémů}{\@department@CS}%
 \ifEqString{#1}{UITS}{inteligentních systémů}{\@department@SK}%
 \ifEqString{#1}{UITS}{Intelligent Systems}{\@department@EN}%
 \ifEqString{#1}{UPGM}{počítačové grafiky a multimédií}{\@department@CS}%
 \ifEqString{#1}{UPGM}{počítačové grafiky a multimédií}{\@department@SK}%
 \ifEqString{#1}{UPGM}{Computer Graphics and~Multimedia}{\@department@EN}%
 \ifnasel
 \else
   \typeout{Spatny ustav! Moznosti: UPSY, UIFS, UITS, UPGM}\set@department{UIFS}
 \fi
}

%Nazev ustavu (CS, SK, EN)
\def\@departmentname@CS{\@dep@CS\ \@department@CS}
\def\@departmentname@SK{\@dep@SK\ \@department@SK}
\def\@departmentname@EN{\@dep@EN\ \@department@EN}

%Nazev prace (v jazyce prace)
\iftoggle{slovak_used}{
  \def\@title{\@title@CS}
}{
  \def\@title{\ifczech\@title@CS\else\@title@EN\fi}
}

%Jmeno, prijmeni, tituly autora
\def\get@author{\ifx\undefined\@author@tp\else%
\@author@tp~\fi\MakeUppercase{\@author}\ifx\undefined\@author@ta\else,~\@author@ta\fi}
%Jmeno, prijmeni, tituly vedouciho
\def\get@supervisor{\ifx\undefined\@supervisor@tp\else%
\@supervisor@tp~\fi\MakeUppercase{\@supervisor}\ifx\undefined\@supervisor@ta\else,~\@supervisor@ta\fi}

%-----------------------------------------------------------------------------
% Projectinfo - parametrizace sablony
%-----------------------------------------------------------------------------
%Parametry 
\define@key{projinfo}{title.cs}{\global\def\@title@CS{#1}}
\define@key{projinfo}{title.en}{\global\def\@title@EN{#1}}
\define@key{projinfo}{department}{\set@department{#1}}
\define@key{projinfo}{project}{\set@project{#1}}
\define@key{projinfo}{year}{\def\@year{#1}}
\define@key{projinfo}{date}{\def\@date{#1}}
\define@key{projinfo}{author}{\def\@author{#1}}
\define@key{projinfo}{author.name}{\def\@author@n{#1}}
\define@key{projinfo}{author.surname}{\def\@author@s{#1}}
\define@key{projinfo}{author.title.a}{\def\@author@ta{#1}}
\define@key{projinfo}{author.title.p}{\def\@author@tp{#1}}
\define@key{projinfo}{supervisor}{\def\@supervisor{#1}}
\define@key{projinfo}{supervisor.name}{\def\@supervisor@n{#1}}
\define@key{projinfo}{supervisor.surname}{\def\@supervisor@s{#1}}
\define@key{projinfo}{supervisor.title.a}{\def\@supervisor@ta{#1}}
\define@key{projinfo}{supervisor.title.p}{\def\@supervisor@tp{#1}}
\define@key{projinfo}{keywords.cs}{\def\@tkeywords@CS{#1}}
\define@key{projinfo}{keywords.en}{\def\@tkeywords@EN{#1}}
\define@key{projinfo}{abstract.cs}{\def\@tabstract@CS{#1}}
\define@key{projinfo}{abstract.en}{\def\@tabstract@EN{#1}}
\define@key{projinfo}{declaration}{\def\@declaration{#1}}
\define@key{projinfo}{acknowledgment}{\def\@acknowledgment{#1}}

%Vychozi hodnoty
\setkeys{projinfo}{
  title.cs={Název práce (česky)},title.en={Název práce(anglicky)}, 
  date=\today, project=BP, department=UIFS, declaration=, 
  abstract.cs=, abstract.en=, keywords.cs=, keywords.en=,
  author=Jmeno Prijmeni, supervisor=Jmeno Prijmeni,
  author.name=, author.surname=, supervisor.name=, 
  supervisor.surname=
}%

\def\projectinfo{\setkeys{projinfo}}

%Prohlaseni
\newcommand{\declaration}[1]{\def\@declaration{#1}}

%Podekovani
\newcommand{\acknowledgment}[1]{\def\@acknowledgment{#1}}

%Abstrakt
\renewcommand{\abstract}[2][cs]{%
 \ifEqString{#1}{cs}{#2}{\@tabstract@CS}%
 \ifEqString{#1}{sk}{#2}{\@tabstract@SK}%
 \ifEqString{#1}{en}{#2}{\@tabstract@EN}%
}

%Klicova slova
\newcommand{\keywords}[2][cs]{%
 \ifEqString{#1}{cs}{#2}{\@tkeywords@CS}%
 \ifEqString{#1}{sk}{#2}{\@tkeywords@SK}%
 \ifEqString{#1}{en}{#2}{\@tkeywords@EN}%
}

\newlength{\mkspc}
\setlength{\mkspc}{2pt}

\def\setfontsize#1{\renewcommand{\baselinestretch}{1.5}\fontsize{#1}{\f@baselineskip}\selectfont}
%text
\newcommand{\mktxt}[2]{\vbox{\setfontsize{#1}\ifVAFLE\usefont{T1}{VafleVUT}{m}{n}\fi\ifOPEN\usefont{T1}{OpenSans}{m}{n}\fi\ifARIAL\usefont{T1}{arial}{m}{n}\fi\noindent#2}\vspace\mkspc}
%bold
\newcommand{\mktxt@b}[2]{\vbox{\setfontsize{#1}\ifVAFLE\usefont{T1}{VafleVUT}{b}{n}\fi\ifOPEN\usefont{T1}{OpenSans}{b}{n}\fi\ifARIAL\usefont{T1}{arial}{b}{n}\fi\noindent\bf#2}\vspace\mkspc}
\newcommand{\mkuctxt}[2]{\vbox{\setfontsize{#1}\ifVAFLE\usefont{T1}{VafleVUT}{m}{n}\fi\ifOPEN\usefont{T1}{OpenSans}{m}{n}\fi\ifARIAL\usefont{T1}{arial}{m}{n}\fi\noindent\MakeUppercase{#2}}\vspace\mkspc}
%text
\newcommand{\mktxtt}[2]{\vbox{\setfontsize{#1}\ifTVAFLE\usefont{T1}{VafleVUT}{m}{n}\fi\ifTOPEN\usefont{T1}{OpenSans}{m}{n}\fi\ifTARIAL\usefont{T1}{arial}{m}{n}\fi\ifTBOPEN\usefont{T1}{OpenSans}{b}{n}\fi\noindent#2}\vspace\mkspc}
%bold
\newcommand{\mktxtt@b}[2]{\vbox{\setfontsize{#1}\ifTVAFLE\usefont{T1}{VafleVUT}{b}{n}\fi\ifTOPEN\usefont{T1}{OpenSans}{b}{n}\fi\ifTARIAL\usefont{T1}{arial}{b}{n}\fi\ifTBOPEN\usefont{T1}{OpenSans}{b}{n}\fi\noindent\bf#2}\vspace\mkspc}
\newcommand{\mkuctxtt}[2]{\vbox{\setfontsize{#1}\ifTVAFLE\usefont{T1}{VafleVUT}{m}{n}\fi\ifTOPEN\usefont{T1}{OpenSans}{m}{n}\fi\ifTARIAL\usefont{T1}{arial}{m}{n}\fi\ifTBOPEN\usefont{T1}{OpenSans}{b}{n}\fi\noindent\MakeUppercase{#2}}\vspace\mkspc}

%-----------------------------------------------------------------------------
%Abstrakt, klicovaslova \makeabstract
%-----------------------------------------------------------------------------
\newcommand\makeabstract{%
\thispagestyle{empty}\null%

\ifczech
\mktxt@b{16}{\@abstract@CS}\noindent\@tabstract@CS
\else
  \ifslovak
    \mktxt@b{16}{\@abstract@SK}\noindent\@tabstract@CS
  \else
    \mktxt@b{16}{\@abstract@EN}\noindent\@tabstract@EN
  \fi
\fi

\vfill

\ifczech
\mktxt@b{16}{\@abstract@EN}\noindent\@tabstract@EN
\else
  \ifslovak
      \mktxt@b{16}{\@abstract@EN}\noindent\@tabstract@EN
    \else
      \mktxt@b{16}{\@abstract@CS}\noindent\@tabstract@CS
    \fi
\fi

\vfill

\ifczech
\mktxt@b{16}{\@keywords@CS}\noindent\@tkeywords@CS
\else
  \ifslovak
    \mktxt@b{16}{\@keywords@SK}\noindent\@tkeywords@CS
  \else
    \mktxt@b{16}{\@keywords@EN}\noindent\@tkeywords@EN
  \fi
\fi

\vfill

\ifczech
\mktxt@b{16}{\@keywords@EN}\noindent\@tkeywords@EN
\else
  \ifslovak
    \mktxt@b{16}{\@keywords@EN}\noindent\@tkeywords@EN
  \else
    \mktxt@b{16}{\@keywords@CS}\noindent\@tkeywords@CS
  \fi
\fi

\vfill

\mktxt@b{16}{\ifslovak\@cite@SK\else\ifczech\@cite@CS\else\@cite@EN\fi\fi}
\iftoggle{slovak_used}{
  \noindent\MakeUppercase{\@author@s}, \@author@n. \it\@title\rm. Brno, \@year. \@projecttype@SK. \@vut@SK, \@fit@SK. {\@supervisor@SK} {\@supervisor@s} {\@supervisor@n}. 
}{
  \noindent\MakeUppercase{\@author@s}, \@author@n. \it\@title\rm. Brno, \@year. \ifczech\@projecttype@CS\else\@projecttype@EN\fi. \ifczech\@vut@CS\else\@vut@EN\fi, \ifczech\@fit@CS\else\@fit@EN\fi. {\ifczech\@supervisor@CS\else\@supervisor@EN\fi} {\@supervisor@s} {\@supervisor@n}. 
}
%===================================================
\newpage
}

%-----------------------------------------------------------------------------
%Prohlaseni, podekovani \makedeclaration
%-----------------------------------------------------------------------------
\newcommand\makedeclaration{
\thispagestyle{empty}%
\mktxt@b{18}{\@title}

\vspace{10mm}
\ifslovak
  \mktxt@b{16}{\@tdecl@SK}
\else
  \mktxt@b{16}{\ifczech\@tdecl@CS\else\@tdecl@EN\fi}
\fi

\noindent\@declaration

  \begin{flushright}
      \makebox[4cm]{\dotfill} \\
      \@author\\
      \@date
  \end{flushright}

\ifx\undefined\@acknowledgment
\else
\vspace{20mm}
\ifslovak
  \mktxt@b{16}{\@tackn@SK}
\else
  \mktxt@b{16}{\ifczech\@tackn@CS\else\@tackn@EN\fi}
\fi
\noindent\@acknowledgment
\fi

\vfill

\noindent\copyright\ \@author, \@year.\newline
{\em \ifslovak\@copyright@SK\else\ifczech\@copyright@CS\else\@copyright@EN\fi\fi}
%===================================================
\newpage
}

%-----------------------------------------------------------------------------
%Uvodni stranky \maketitle
%-----------------------------------------------------------------------------
\renewcommand\maketitle{%
\thispagestyle{empty}\null%
%=====================================================
\fontfamily{qhv}\selectfont%
%\newsavebox{\@obrlogo}\newsavebox{\@rtext}\newlength{\@vyskartext}\newlength{\@vyskaobr}%
\newsavebox{\@obrlogo}\newsavebox{\@rtext}\newlength{\@vyskartext}\newlength{\@sirkartext}\newlength{\@vyskaobr}\newlength{\@sirkaobr}%

\ifJVS
  \ifPrint
    \savebox{\@obrlogo}{\hspace*{-0.6cm}\includegraphics[width=4.1cm,keepaspectratio,trim={1.2cm 1.2cm 1.2cm 1.2cm},clip]{fig/VUT_symbol_cernobile_CZ}}%symbol VUT
  \else
    \savebox{\@obrlogo}{\hspace*{-0.6cm}\includegraphics[width=4.1cm,keepaspectratio,trim={1.2cm 1.2cm 1.2cm 1.2cm},clip]{fig/VUT_symbol_barevne_CMYK_CZ}}%symbol VUT
  \fi
\else
  \savebox{\@obrlogo}{\includegraphics[width=3.5cm,keepaspectratio]{fig/vut-zp2}}%logo VUT
\fi
\savebox{\@rtext}{\vbox{\mkuctxt{18}{\ifslovak\@vut@SK\else\ifczech\@vut@CS\else\@vut@EN\fi\fi}\mkuctxt{11}{\ifczech\@vut@EN\else\ifslovak\@vut@EN\else\@vut@CS\fi\fi}}}%text VUT
\settoheight{\@vyskaobr}{\makebox{\usebox{\@obrlogo}}}% vyska loga
\settowidth{\@sirkaobr}{\makebox{\usebox{\@obrlogo}}}% vyska loga
\settoheight{\@vyskartext}{\makebox{\usebox{\@rtext}}}% vyska textu
\settowidth{\@sirkartext}{\makebox{\usebox{\@rtext}}}% sirka textu
\addtolength{\@vyskaobr}{-\@vyskartext}%odecteme od vysky loga vysku textu
\addtolength{\@sirkartext}{-\@sirkaobr}\addtolength{\@sirkartext}{-10pt}%sirka textu na radku s obrazkem a odsazenim

\newtoggle{gray_used}
\togglefalse{gray_used}

\ifGRAY
  \usebox{\@obrlogo}
  
  \vspace{10mm}
  
  \mkuctxtt{18}{\ifczech\@vut@CS\else\@vut@EN\fi}
  \mkuctxtt{11}{\textcolor{VUTGRAY}{\ifczech\@vut@EN\else\@vut@CS\fi}}

  \vspace{5mm}
  
  \mkuctxtt{14}{\ifczech\@fit@CS\else\@fit@EN\fi}
  \mkuctxtt{11}{\textcolor{VUTGRAY}{\ifczech\@fit@EN\else\@fit@CS\fi}}

  \vspace{5mm}
  
  \mkuctxtt{14}{\ifczech\@departmentname@CS\else\@departmentname@EN\fi}
  \mkuctxtt{11}{\textcolor{VUTGRAY}{\ifczech\@departmentname@EN\else\@departmentname@CS\fi}}

  \toggletrue{gray_used}
\fi

\iftoggle{gray_used}{
}{
  \ifJVS
    \usebox{\@obrlogo}
  
    \vspace{10mm}
  
    \iftoggle{slovak_used}{
      \mkuctxtt{18}{\@vut@SK}
      \mkuctxtt{11}{\@vut@EN}
    }{
      \mkuctxtt{18}{\ifczech\@vut@CS\else\@vut@EN\fi}
      \mkuctxtt{11}{\ifczech\@vut@EN\else\@vut@CS\fi}
    }

    \vspace{5mm}
  
    \iftoggle{slovak_used}{
      \mkuctxtt{14}{\@fit@SK}
      \mkuctxtt{11}{\@fit@EN}
    }{
      \mkuctxtt{14}{\ifczech\@fit@CS\else\@fit@EN\fi}
      \mkuctxtt{11}{\ifczech\@fit@EN\else\@fit@CS\fi}
    }

    \vspace{5mm}
  
    \iftoggle{slovak_used}{
      \mkuctxtt{14}{\@departmentname@SK}
      \mkuctxtt{11}{\@departmentname@EN}
    }{
      \mkuctxtt{14}{\ifczech\@departmentname@CS\else\@departmentname@EN\fi}
      \mkuctxtt{11}{\ifczech\@departmentname@EN\else\@departmentname@CS\fi}
    }
  \else
    \noindent\usebox{\@obrlogo}\raisebox{\@vyskaobr}{\hspace{10pt}\usebox{\@rtext}}%zvedneme box s textem
  \fi
}

\vspace{5mm}

\ifJVS
\else
\savebox{\@obrlogo}{\includegraphics[width=3.5cm,keepaspectratio]{fig/fit-zp2}}%logo FIT

\iftoggle{slovak_used}{
\savebox{\@rtext}{%text FIT, USTAV
  \vbox{\mkuctxt{14}{\@fit@CS}
  	\noindent\parbox[l]{\@sirkartext}{{\setfontsize{14}\noindent\RaggedRight\MakeUppercase{\@departmentname@SK}}\vspace\mkspc}
  	\vspace{3mm}\linebreak
  	\mkuctxt{11}{\@fit@EN}
  	\mkuctxt{11}{\@departmentname@EN}
  	}
     }%
}{
\savebox{\@rtext}{%text FIT, USTAV
  \vbox{\mkuctxt{14}{\ifczech\@fit@CS\else\@fit@EN\fi}
%  	  \mkuctxt{14}{\ifczech\@departmentname@CS\else\@departmentname@EN\fi}\\
  	\noindent\parbox[l]{\@sirkartext}{{\setfontsize{14}\noindent\RaggedRight\MakeUppercase{\ifczech\@departmentname@CS\else\@departmentname@EN\fi}}\vspace\mkspc}
  	\vspace{3mm}\linebreak
  	\mkuctxt{11}{\ifczech\@fit@EN\else\@fit@CS\fi}
  	\mkuctxt{11}{\ifczech\@departmentname@EN\else\@departmentname@CS\fi}
  	}
     }%
}
\settoheight{\@vyskaobr}{\makebox{\usebox{\@obrlogo}}}% vyska loga
\settoheight{\@vyskartext}{\makebox{\usebox{\@rtext}}}% vyska textu
\addtolength{\@vyskaobr}{-\@vyskartext}%odecteme od vysky loga vysku textu
\noindent\usebox{\@obrlogo}\raisebox{\@vyskaobr}{\hspace{10pt}\usebox{\@rtext}}%zvedneme box s textem
\fi

\vfill

\mkuctxtt{18}{\ifslovak\@title@CS\else\ifczech\@title@CS\else\@title@EN\fi\fi}
\iftoggle{gray_used}{
  \mkuctxtt{10}{\textcolor{VUTGRAY}{\ifslovak\@title@EN\else\ifczech\@title@EN\else\@title@CS\fi\fi}}
}{
  \mkuctxtt{10}{\ifslovak\@title@EN\else\ifczech\@title@EN\else\@title@CS\fi\fi}
}

\vfill

\mkuctxtt{14}{\ifslovak\@projecttype@SK\else\ifczech\@projecttype@CS\else\@projecttype@EN\fi\fi}
\iftoggle{gray_used}{
  \mkuctxtt{10}{\textcolor{VUTGRAY}{\ifslovak\@projecttype@EN\else\ifczech\@projecttype@EN\else\@projecttype@CS\fi\fi}}
}{
  \mkuctxtt{10}{\ifslovak\@projecttype@EN\else\ifczech\@projecttype@EN\else\@projecttype@CS\fi\fi}
}

\vspace{5mm}

\mktxtt{14}{\MakeUppercase{\ifslovak\@author@SK\else\ifczech\@author@CS\else\@author@EN\fi\fi} \hfill \get@author}
\iftoggle{gray_used}{
  \mkuctxtt{10}{\textcolor{VUTGRAY}{\ifslovak\@author@EN\else\ifczech\@author@EN\else\@author@CS\fi\fi}}
}{
  \mkuctxtt{10}{\ifslovak\@author@EN\else\ifczech\@author@EN\else\@author@CS\fi\fi}
}


\vspace{5mm}

\mktxtt{14}{\MakeUppercase{\ifslovak\@supervisor@SK\else\ifczech\@supervisor@CS\else\@supervisor@EN\fi\fi} \hfill \get@supervisor}
\iftoggle{gray_used}{
  \mkuctxtt{10}{\textcolor{VUTGRAY}{\ifslovak\@supervisor@EN\else\ifczech\@supervisor@EN\else\@supervisor@CS\fi\fi}}
}{
  \mkuctxtt{10}{\ifslovak\@supervisor@EN\else\ifczech\@supervisor@EN\else\@supervisor@CS\fi\fi}
}


\vspace{10mm}

\mktxtt{11}{BRNO \@year}

%zadani
\ifzadani\includepdf[pages=-,offset=0.6cm -1.7cm]{zadani.pdf}\fi

\newpage\fontfamily{\familydefault}\selectfont%
%=================================================================================
\makeabstract\makedeclaration
\clearpage
\setcounter{tocdepth}{2}\setcounter{page}{1}
}

%-----------------------------------------------------------------------------
\def\uv#1{\iflanguage{english}{``#1''}%
                              {\quotedblbase #1\textquotedblleft}}%
%\catcode`\"=\active
%\def"{\bgroup
%    \iflanguage{english}%
%        {``\def"{''\egroup}}%
%        {\quotedblbase\def"{\textquotedblleft\egroup}}}

% Now, switch on what is appropriate for czech:

% czech quotation marks
% \bq - begin quotation, \eq - end quotation
\def\bq{\mbox{\kern.1ex\protect\raisebox{-1.3ex}[0pt][0pt]{''}\kern-.1ex}}
\def\eq{\mbox{\kern-.1ex``\kern.1ex}}
%\setlanguage{\czech}

{%                                      % Begin a group for which " is active
\catcode`\"=\active                     % Make " an active character
\catcode`\@=11                          % Make @ an active character
%
%  \csdoublequoteson
%
%       This macro makes " an active character, resets the control sequence
%       \dblqu@te to L (left), and defines \dq as a replacement for ".
%
\gdef\csdoublequoteson{%                % \csdoublequoteson enables "
    \gdef"{\czechquotes}%               % Define " as \czechquotes
    \global\catcode`\"=\active%         % Make " an active character
    \global\chardef\dq=`\"%             % Double-quote char. via \dq
    \global\let\dblqu@te=L%             % Always start with a left double-quote
    }                                   % End of macro
%
%  \bq, \eq
%
%      These macros define default characters for czech left and right
%      double quotes. Czech opening quote is created from two commas with
%      kerning depending on fontdimen four parameter of current font.
%      Better solution should be specially designed character with
%      proper kernings; if you have such characters in fonts
%      (e.g. in DC-fonts), use it instead. (e.g. define
%      macros \bq and \eq e.g. \def\bq{\char"130 }
%      in your document/style file-- not in csquote.sty!)
%      Similar solution is used for czech right quote.
%
%      \cs existence test, stolen from TeXbook exercise 7.7
\def\ifundefined#1{\expandafter\ifx\csname#1\endcsname\relax }%
%
%      another macro to be more efficient in time and space
\global\chardef\f@@r=4
%
\ifundefined{bq}%
\gdef\bq{\kern-.25\fontdimen\f@@r\font,\kern-.8\fontdimen\f@@r\font,%
                \kern-.35\fontdimen\f@@r\font}%
\fi
\ifundefined{eq}%
\gdef\eq{\kern-.35\fontdimen\f@@r\font`\kern-.8\fontdimen\f@@r\font`%
                \kern-.25\fontdimen\f@@r\font}
\fi
%
% Macro \uv for other usage of \bq and \eq.
%
\ifundefined{uv}\iflanguage{czech}%
        \gdef\uv#1{\bq #1\eq}
\fi
%
% \testquotes macro gives warning if citation span this place
%
\gdef\testquotes{\if R\dblqu@te
        \message{Warning: You forgot right double quote!}%
        \let\dblqu@te=L\fi}
%
%  Define the macro that will be executed whenever " is encountered.
%
\gdef\czechquotes{\protect\czechqu@tes}
\gdef\czechqu@tes{%
        %  If the double-quote is the first character in a new paragraph,
        %  make sure it becomes a left double-quote.  This case can be
        %  detected by checking to see if TeX is currently in vertical mode.
        %  If so, the double-quote is at the beginning of the paragraph
        %  (since " hasn't actually generated any horizontal mode tokens
        %  yet, TeX is still in vertical mode).  If the mode is vertical,
        %  set \dblqu@te equal to L.
        %
        \ifinner\else\ifvmode\testquotes\fi\fi%
        %
        %  Now insert the appropriate left or right double-quote.
        %
        %  If \dblqu@te is L, insert an opening quote and set \dblqu@te to R.
        %
        \if L\dblqu@te\bq\global\let\dblqu@te=R%
        %
        %  Otherwise, save the current \spacefactor, insert '', set \dblqu@te
        %  to L, and reset the original \spacefactor.
        %
        \else%
           \let\xxx=\spacefactor%               % Save the \spacefactor
           \eq%                                 % Insert ending quote
           \global\let\dblqu@te=L%              % and reset \dblqu@te
           \spacefactor\xxx%                    % Reset the \spacefactor
        \fi%                                    % End of \if L\dblqu@te...
        }                                       % End of " macro
}                                               % End of group

\gdef\csdoublequotesoff{%
        \catcode`\"=12%                         % Set " back to other
        }
%
% Czech quotes are default
%
\csdoublequoteson




\endinput
%%
%% End of file `fitthesis.cls'.
