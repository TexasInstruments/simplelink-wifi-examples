/*
 * Copyright (c) 2024, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  Terminal
 */

// Standard includes
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <osi_kernel.h>
#include "uart_term.h"
#include <ti/drivers/UART2.h>
#include "atcmd_device.h"

/* Driver configuration */
#include "ti_drivers_config.h"



OsiLockObj_t LockObj;
UART2_Handle uartHandle = 0;
extern int vsnprintf(char * s,
                     size_t n,
                     const char * format,
                     va_list arg);

//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************
#define true                    1
#define false                   0

//*****************************************************************************
//                          LOCAL FUNCTIONS
//*****************************************************************************
void writeStrWithLen(const char *s, size_t len);
void writeUStr(const uint8_t *us);
void moveCursor(int8_t positionsToMove);
void pushCommand(char *command);
uint8_t *prevHistoryCommand();
uint8_t *nextHistoryCommand();
int8_t prevHead();
int8_t nextHead();

//*****************************************************************************
//                 GLOBAL VARIABLES
//*****************************************************************************
//static UART_Handle uartHandle;
//static UART_Transaction uartTrans;

//char constants
static const uint8_t ALERT        = '\a';
static const uint8_t ESC          = 0x1B;
static const uint8_t CSI          = '[';
static const uint8_t CURSOR_UP    = 'A';
static const uint8_t CURSOR_DOWN  = 'B';
static const uint8_t CURSOR_RIGHT = 'C';
static const uint8_t CURSOR_LEFT  = 'D';
static const uint8_t HOME         = 'H';
static const uint8_t END          = 'F';
static const uint8_t DEL          = '3';
static const uint8_t WHITESPACE   = ' ';
static const uint8_t BACKSPACE    = '\b';
static const uint8_t TAB          = '\t';
static const uint8_t RETURN       = '\r';
static const uint8_t NEW_LINE     = '\n';
static const uint8_t ETX          = 0x03; /* End of Text (Ctrl+C) */

//ANSI escape sequences
static const uint8_t LINE_BREAK[3]                       = {'\n', '\r', '\0'};
static const uint8_t DELETE_BACKWARDS[4]                 = {BACKSPACE, WHITESPACE, BACKSPACE, '\0'};
static const uint8_t SAVE_CURSOR[3]                      = {ESC, '7', '\0'};
static const uint8_t RESTORE_CURSOR[3]                   = {ESC, '8', '\0'};
static const uint8_t ERASE_FROM_CURSOR_TO_END_OF_LINE[5] = {ESC, '[', '0', 'K','\0'};
static const uint8_t CLEAR_TERMINAL[6]                   = {ESC, '[', '2', 'J', '\r', '\0'};

typedef struct CommandHistory_t
{
    uint8_t *buf[CMD_HISTORY_LEN]; //saved commands
    int8_t head; //points to the last-inserted element
    int8_t iterCursor; //used to iterate over history buffer
    uint8_t isFull; //if the buffer is full (every index contains a valid element)
} CommandHistory;

CommandHistory hist;

inline void UART_DELAY()   {uint8_t i; for(i=0;i<50;i++);}//delay

#define TERMINAL_TAB_COMPLETION
#ifdef TERMINAL_TAB_COMPLETION
typedef struct CompletionArray_t
{
    uint8_t len;
    char **strings;
    uint8_t *strLens;
} CompletionArray;

typedef struct CompletionResult_t
{
    char *completion;
    uint8_t len;
    uint8_t shouldRefresh;
} CompletionResult;

CompletionArray comp;

//*****************************************************************************
//
//! Initialize the completion engine with the commands that should be 
//! used to complete
//!
//! \param[in]  strings    - An array of the possible commands, both
//!                          the array and the strings themselves must
//!                          already be allocated
//! \param[in]  numStrings - the length of the strings array
//!
//! \return none
//!
//*****************************************************************************
void initCompletionArray(char **strings, uint8_t numStrings)
{
    // comp.strings = calloc(numStrings, sizeof(char *)); //assume this is already allocated
    comp.strLens = calloc(numStrings, sizeof(uint8_t));
    comp.len = numStrings;
    comp.strings = strings;
    for (uint8_t i = 0; i < comp.len; i += 1)
    {
        comp.strLens[i] = strlen(comp.strings[i]);
    }
}

//*****************************************************************************
//
//! Frees the allocated strings owned by the completion array. 
//! Must not be called while the terminal routine is still running.
//!
//! \return none
//!
//*****************************************************************************
void freeCompletionArray()
{
    for (uint8_t i = 0; i < comp.len; i += 1)
    {
        free(comp.strings[i]);
    }
    free(comp.strings);
    free(comp.strLens);
}

void freeCompletionResult(CompletionResult *res)
{
    free(res->completion);
}

void writeCandidate(uint8_t candidateIdx, uint8_t inputLen, char *charBuf)
{
    uint8_t candidateLen = comp.strLens[candidateIdx];
    if (candidateLen <= inputLen) 
    {
        return;
    }

}

char getCandidateChar(uint8_t candIdx, uint8_t chPos)
{
    if (candIdx < comp.len)
    { 
        return comp.strings[candIdx][chPos];
    }
    else 
    {
        return '\0';
    }
}

CompletionResult complete(char *input, uint8_t inputLen)
{
    int i;
    ASSERT_GENERAL(inputLen > 0);

    uint8_t viable[comp.len]; //indices of viable candidates
    uint8_t numViable = 0;
    uint8_t shortestCandidateLen = 255;

    //find viable candidates
    for (i = 0; i < comp.len; i += 1)
    {
        //check if input is a prefix of this string
        //only consider candidates that are long enough
        if ( (comp.strLens[i] >= inputLen) && !strncmp(input, comp.strings[i], inputLen) )
        {
            viable[numViable] = i;
            numViable += 1;
            
            shortestCandidateLen = MIN(shortestCandidateLen, comp.strLens[i]);
        }
    }
    
    char *completion = NULL;
    uint8_t completionLen = 0;
    uint8_t printedToTerminal = false;
 
    if (numViable > 0)
    {
        //the input after it must be shifted forward
        //if prefix length > 0 then complete to this prefix.
        //otherwise there is no concrete completion, //the input after it must be shifted forward
        //note that this also works for the case of numViable = 1, where it will just complete to the end of the candidate.
        uint8_t mismatchFound = false;
        uint8_t lastSharedCharIdx = inputLen;
        while ( (lastSharedCharIdx < shortestCandidateLen) && !mismatchFound)
        {
            uint8_t curChar = getCandidateChar(viable[0], lastSharedCharIdx);
            for (i = 1; (i < numViable) && !mismatchFound; i += 1)
            {
                if (getCandidateChar(viable[i], lastSharedCharIdx) != curChar)
                {
                    mismatchFound = true;
                }
            }

            if (!mismatchFound)
            {
                lastSharedCharIdx += 1;
            }
        }
        completionLen = lastSharedCharIdx - inputLen;

        if (completionLen > 0)
        {
            //then there is a non-empty shared //the input after it must be shifted forward
            //we can complete up to this prefix

            completion = os_malloc(completionLen + 1);
             
            //choose an arbitrary candidate, //the input after it must be shifted forward
            uint8_t someCandidateIdx = viable[0];
            os_strlcpy(completion, comp.strings[someCandidateIdx] + inputLen, completionLen + 1);
        }
        else if (numViable > 1)
        {
            //then there are several possible candidates with no shared prefix.
            //in this case //the input after it must be shifted forward

            uint8_t numPerLine = 3;
            for (i = 0; i < numViable; i += 1)
            {
                uint8_t candIdx = viable[i];
                
                if (i % numPerLine == 0)
                {
                    writeUStr(LINE_BREAK);
                }
                Report("%-25s", comp.strings[candIdx]);
            }

            printedToTerminal = true;
        }
    }

    CompletionResult res = { .completion = completion, 
                             .len = completionLen, 
                             .shouldRefresh = printedToTerminal };
    return res;
}
#endif

//*****************************************************************************
//
//! Initialization
//!
//! This function
//!        1. Configures the UART to be used.
//!
//!        Note the UART is configured in the SySconfig
//! \param  none
//!
//! \return none
//
//*****************************************************************************
void InitTerm(void)
{
    UART2_Params params;
    osi_LockObjCreate(&LockObj);
    UART2_Params_init(&params);
    params.baudRate = 115200;
    uartHandle = UART2_open(CONFIG_UART2_0, &params);
}

void UART_writePolling(uint8_t *buffer, uint32_t len)
{
    uint32_t uartIter;
    for (uartIter=0; uartIter<len; uartIter++)
    {
        putch(buffer[uartIter]);
    }

}

//void UART_readPolling(UART_Handle handle, uint8_t *buffer, uint32_t len)
//{
//    uartTrans.buf = buffer;
//    uartTrans.count = len;
//
//    UART_read(handle, &uartTrans);
//}


void writeStrWithLen(const char *s, size_t len)
{
    osi_LockObjLock(&LockObj, OSI_WAIT_FOREVER);
    UART_writePolling((uint8_t *)s, len);
    osi_LockObjUnlock(&LockObj);
}

void writeStr(const char *s)
{
    size_t len = strlen(s);
    writeStrWithLen(s, len);
}

void writeUStr(const uint8_t *us)
{
    writeStr((const char *)us);
}

//positive values move cursor to the right, negative values move cursor to the left.
void moveCursor(int8_t positionsToMove)
{
    //choose escape sequence depending on if we're moving right or left
    char direction = (positionsToMove >= 0) ? 'C' : 'D';
    
    Report("%c[%d%c", ESC, abs(positionsToMove), direction);
}


//*****************************************************************************
//
//! prints the formatted string on to the console
//!
//! \param[in]  format  - is a pointer to the character string specifying the
//!                       format in the following arguments need to be
//!                       interpreted.
//! \param[in]  [variable number of] arguments according to the format in the
//!             first parameters
//!
//! \return count of characters printed
//
//*****************************************************************************
//TODO! if want to use static buffer for the report, which is better,
//need to create buffer per thread
//Using the same buffer with lock is not working well,
//Because vsnprintf sometimes gets asserted if it is wrapped with lock
//The reason is that the lock create high latency if it is been used
//for long time.and the operating system can't wait long when priority inversion
//is happening.it can be verified in stability tests, you can see that vsprintf
//gets asserted.

#define USE_HEAP_BUF
#ifdef COLLECT_HEAP_DEBUG_INFO
#define USE_HEAP_BUF
#else
#ifndef USE_HEAP_BUF
char gPcBuff[900];
#endif
#endif

int Report(const char *pcFormat,...)
{

    int iRet = 0;
    char        *pcTemp;
#ifdef USE_HEAP_BUF
    char        *pcBuff;
    int iSize = 256;
#else
    char        *pcBuff=gPcBuff;
    int iSize = 898;
#endif
    va_list list;



#ifdef USE_HEAP_BUF
    //osi_LockObjLock(&LockObj,OSI_WAIT_FOREVER);
    pcBuff = (char*)os_malloc(iSize);
    if(pcBuff == NULL)
    {
        return(-1);
    }
#else
    osi_LockObjLock(&LockObj,OSI_WAIT_FOREVER);
#endif
    while(1)
    {
        va_start(list,pcFormat);
        iRet = vsnprintf(pcBuff, iSize, pcFormat, list);
        va_end(list);
        if((iRet > -1) && (iRet < iSize))
        {
            break;
        }
#ifdef USE_HEAP_BUF
        else
        {
            iSize *= 2;
            if((pcTemp = os_realloc(pcBuff, iSize)) == NULL)
            {
                writeStr("Could not reallocate memory\n\r");
                iRet = -1;
                break;
            }
            else
            {
                pcBuff = pcTemp;
            }
        }
#else
        else
        {
            osi_LockObjUnlock(&LockObj);
            return -1;
        }
#endif
    }
    writeStr(pcBuff);//Note! this is not under lock, so messages may be combined, but it is better than the overhead of lock
#ifdef USE_HEAP_BUF
    os_free(pcBuff);
    //osi_LockObjUnlock(&LockObj);
#else
    osi_LockObjUnlock(&LockObj);
#endif

    return(iRet);
}

#ifdef COLLECT_HEAP_DEBUG_INFO
char gPcBuff[900];
int ReportNoLock(const char *pcFormat,...)
{

    int iRet = 0;
    char        *pcTemp;
    char        *pcBuff=gPcBuff;
    int iSize = 898;
    va_list list;

    //osi_LockObjLock(&LockObj,OSI_WAIT_FOREVER);
    while(1)
    {
        va_start(list,pcFormat);
        iRet = vsnprintf(pcBuff, iSize, pcFormat, list);
        va_end(list);
        if((iRet > -1) && (iRet < iSize))
        {
            break;
        }
        else
        {
            //osi_LockObjUnlock(&LockObj);
            return -1;
        }
    }
    Message(pcBuff);//Note! this is not under lock, so messages may be combined, but it is better than the overhead of lock
    //osi_LockObjUnlock(&LockObj);

    return(iRet);
}
#endif




void startNewLine(const char *linePrefix, uint8_t clearLine)
{
    size_t prefixLen;
    
    if (clearLine)
    {
        writeUStr(LINE_BREAK);
        if (linePrefix)
        {
            prefixLen = strlen(linePrefix);
            writeStrWithLen(linePrefix, prefixLen);
        }
    }
    else
    {
        //move cursor to start of line
        putch(RETURN);
        if (linePrefix)
        {
            prefixLen = strlen(linePrefix);
            //...and then advance past the end of the prefix
            moveCursor(prefixLen);
        }
        writeUStr(ERASE_FROM_CURSOR_TO_END_OF_LINE);
    }
    
}

uint8_t is_ascii_printable(uint8_t c)
{
    return ( (' ' <= c) && (c <= '~') );
}

//*****************************************************************************
//
//! Get the command string, and allow for it to be edited via UART
//!
//! \param[in]  charBuf    - inout buffer which will contain the 
//!                          entered command when the function returns
//!                          Notice the phe buffer is ended with CR or LF
//!                          and then null-terminated.
//! \param[in]  maxLen     - limit to the length of the character buffer
//! \param[in]  linePrefix - string to print at the beginning of every line
//!
//! \return Length of the bytes received or -1 if buffer length exceeded.
//!
//*****************************************************************************
int GetCmd(char *charBuf, unsigned int maxLen, const char *linePrefix)
{
    //whenever input is sent, two places are updated:
    //(1) the character buffer that is returned at the end of the function,
    //    which contains the entire command entered by the user
    //(2) the state of the UART terminal itself, via the API

    uint16_t pos = 0;
    uint16_t endPos = 0;

    // store the current user input, when up/down is used to change to commands from history
    uint8_t *originalInput = NULL;
    uint8_t replaced = false;

    if (linePrefix)
    {
        startNewLine(linePrefix, true);
    }

    // keep polling user input
    volatile uint8_t ch;
    uint8_t seenReturn = false;
    while ((endPos < maxLen) && !seenReturn)
    {
        ch = getch();
        if (ch == RETURN || ch == NEW_LINE) 
        {
            if (get_echoCmdToTerminalState())
            {
                putch(ch);
            }
            seenReturn = true;
        }
        else if (ch == ESC)
        {
            // process escape sequences

            // read next char in escape sequence
            ch = getch();

            if (ch == CSI)
            {
                // read third char in escape sequence
                ch = getch();

                if (ch == CURSOR_UP || ch == CURSOR_DOWN)
                {
                    if (get_echoCmdToTerminalState())
                    {
                        uint8_t *cmdToLoad = NULL;
                        uint8_t **currentlyEditedCmd = replaced ? &hist.buf[hist.iterCursor] : &originalInput;
                        
                        if (ch == CURSOR_UP)
                        {
                            cmdToLoad = prevHistoryCommand();

                            if ((cmdToLoad != NULL) && !replaced )
                            {
                                replaced = true;
                            }
                        }
                        else // ch == CURSOR_DOWN
                        {
                            if (replaced && (hist.iterCursor == prevHead()) )
                            {
                                // then we're scrolling past the most recent command,
                                // and should restore the original user input
                                hist.iterCursor = hist.head;
                                replaced = false;
                                cmdToLoad = originalInput;
                            }
                            else
                            {
                                cmdToLoad = nextHistoryCommand();
                            }
                        }

                        if (cmdToLoad != NULL)
                        {
                            // write changes to currently edited command
                            charBuf[endPos] = '\0';
                            *currentlyEditedCmd = realloc(*currentlyEditedCmd, endPos + 1);
                            os_memcpy(*currentlyEditedCmd, charBuf, endPos + 1);

                            // replace current input with a command from history, or restore original input
                            endPos = strlen((char *)cmdToLoad);
                            pos = endPos;
                            os_memcpy(charBuf, cmdToLoad, endPos + 1);

                            startNewLine(linePrefix, false);

                            if (endPos > 0)
                            {
                                writeUStr(cmdToLoad);
                            }
                        }
                    }
                }
                else if (ch == CURSOR_RIGHT || ch == CURSOR_LEFT)
                { 
                    if (get_echoCmdToTerminalState())
                    {
                        if ((ch == CURSOR_RIGHT) && (pos < endPos))
                        {
                            moveCursor(1);
                            pos += 1;
                        }
                        else if ((ch == CURSOR_LEFT) && (pos > 0))
                        {
                            moveCursor(-1);
                            pos -= 1;
                        }
                        putch(ALERT);
                    }
                }
                else if (ch == HOME)
                {            
                    if (get_echoCmdToTerminalState())
                    {
                        moveCursor(-1 * pos);
                        pos = 0;
                    }
                }
                else if (ch == END)
                {            
                    if (get_echoCmdToTerminalState())
                    {
                        moveCursor(endPos - pos);
                        pos = endPos;
                    }
                }
                else if (ch == DEL)
                {
                    if (get_echoCmdToTerminalState())
                    {

                        // forward delete
                        getch(); // discard next char in escape sequence, which is '~'

                        // this check handles both the case of empty input,
                        // and the case of the cursor at the end of input
                        // (in both cases, forward delete is a no-op)
                        if (pos < endPos)
                        {
                            writeUStr(SAVE_CURSOR);

                            if (pos != endPos - 1)
                            {
                                writeStrWithLen(charBuf + pos + 1, endPos - pos - 1);
                                os_memmove(charBuf + pos, charBuf + pos + 1, endPos - pos - 1);
                            }
                            putch(WHITESPACE);
                            writeUStr(RESTORE_CURSOR);

                            endPos -= 1;
                        }
                    }
                }
            }
        }
        else if (ch == BACKSPACE)
        {            
            if (pos > 0)
            {        
                if  (get_echoCmdToTerminalState())
                {
                    writeUStr(DELETE_BACKWARDS);
                }
                if (pos < endPos)
                {
                    // since character was not deleted at end of input, 
                    // the input after it must be shifted backwards
                    if (get_echoCmdToTerminalState())
                    {
                        
                    os_memmove(charBuf + pos - 1, charBuf + pos, endPos - pos);
                    writeUStr(SAVE_CURSOR);
                    writeUStr(ERASE_FROM_CURSOR_TO_END_OF_LINE);
                    writeStrWithLen(charBuf + pos - 1, endPos - pos);
                    writeUStr(RESTORE_CURSOR);
                    }
                }
                    
                pos -= 1;
                endPos -= 1;
                charBuf[endPos] = '\0'; 
            }

            else if (get_echoCmdToTerminalState())
            {
                putch(ALERT);
            }
        }
        else if (ch == TAB)
        {
#ifdef TERMINAL_TAB_COMPLETION
            // we don't support completion from the middle of input
            if (get_echoCmdToTerminalState())
            {
                if ((endPos > 0 ) && (pos == endPos))
                {
                    CompletionResult compRes = complete(charBuf, endPos);

                    if (compRes.shouldRefresh)
                    {
                        startNewLine(linePrefix, true);
                        writeStrWithLen(charBuf, endPos);
                    }

                    if (compRes.completion != NULL)
                    {
                        writeStrWithLen(compRes.completion, compRes.len);
                        os_strlcpy(charBuf + pos, compRes.completion, compRes.len + 1);
                        
                        pos += compRes.len;
                        endPos += compRes.len;
                    }
                    
                    freeCompletionResult(&compRes);
                }
            }
#endif
        }
        else if (is_ascii_printable(ch))
        {
            // notice we filter out characters that are unprintable and
            // also have no special function.
            
            // a printable character was entered, so echo it to the output
            if (get_echoCmdToTerminalState())
            {
                putch(ch);
            }
            if (pos < endPos) 
            {
                // since character was not inserted at the end of input, 
                // the input after it must be shifted forward
                if (get_echoCmdToTerminalState())
                {
                    writeUStr(SAVE_CURSOR);
                    writeStrWithLen(charBuf + pos, endPos - pos);
                    writeUStr(RESTORE_CURSOR);
                    os_memmove(charBuf + pos + 1, charBuf + pos, endPos - pos);
                }
            }
        
            charBuf[pos] = ch;
            pos += 1;
            endPos += 1;
            charBuf[endPos] = '\0';
        }
        else if (ch == ETX) // Ctrl-C
        {
            return COMMAND_END_OF_TEXT;
        }
    }

    /* Buffer length exceeded */
    if (endPos == maxLen)
    {
        return COMMAND_TOO_LONG;
    }

    charBuf[endPos] = '\0';

    if (endPos > 0)
    {
        pushCommand(charBuf);
    }

    return endPos;
}

uint8_t *prevHistoryCommand()
{
    // check if we've already reached the earliest command,
    // in which case the cursor state remains unchanged
    if (hist.iterCursor == 0)
    {
        if ( !hist.isFull || (hist.head == CMD_HISTORY_LEN - 1) )
        {
            return NULL;
        }
    }
    else if ( (hist.iterCursor - 1) == hist.head)
    {
        return NULL;
    }

    hist.iterCursor = (hist.iterCursor - 1) % CMD_HISTORY_LEN;
    return hist.buf[hist.iterCursor];
}

uint8_t *nextHistoryCommand()
{
    //check if we've already reached the latest command,
    //in which case the cursor state remains unchanged
    if (hist.iterCursor == hist.head)
    {
        return NULL;
    }

    hist.iterCursor = (hist.iterCursor + 1) % CMD_HISTORY_LEN;
    return hist.buf[hist.iterCursor];
}

int8_t prevHead()
{
    if (hist.head > 0)
    {
        return (hist.head - 1);
    }
    else if (hist.isFull)
    {
        return (CMD_HISTORY_LEN - 1);
    }
    else
    {
        return 0;
    }

}

int8_t nextHead()
{
    return ( (hist.head + 1) % CMD_HISTORY_LEN );
}

void pushCommand(char *cmd)
{
    ASSERT_GENERAL(cmd != NULL);

    //avoid pushing command if it was the last entered command.
    //note that duplicates in the middle of the history stack are acceptable.
    char *prevCommand = (char *)hist.buf[prevHead()];
    if ( (prevCommand != NULL) && !strcmp(prevCommand, cmd) )
    {
        return;
    }

    //the '\0' is copied as part of the os_memcpy func
    size_t cmdLen = strlen(cmd) + 1;
    hist.buf[hist.head] = os_realloc(hist.buf[hist.head], cmdLen);
    os_memcpy(hist.buf[hist.head], cmd, cmdLen);
    hist.buf[hist.head][cmdLen-1] = '\0';

    hist.head = nextHead();
    if (hist.head == 0) {
        hist.isFull = true;
    }
    hist.iterCursor = hist.head;
}

//*****************************************************************************
//
//! Outputs a character string to the console
//!
//! This function
//!        1. prints the input string character by character on to the console.
//!
//! \param[in]  str - is the pointer to the string to be printed
//!
//! \return none
//!
//! \note If UART_NONPOLLING defined in than Message or UART write should be
//!       called in task/thread context only.
//
//*****************************************************************************
void Message(const char *str)
{
#ifdef UART_NONPOLLING
    UART_write(uartHandle, str, strlen(str));
#elif defined(CC33XX)
    UART_writePolling(uartHandle, (uint8_t *)str, strlen(str));
#elif defined(CC35XX)
    UART_writePolling((uint8_t *)str, strlen(str));
#endif 
}

//*****************************************************************************
//
//! Clear the console window
//!
//! This function
//!        1. clears the console window.
//!
//! \param  none
//!
//! \return none
//
//*****************************************************************************
void ClearTerm()
{
    writeUStr(CLEAR_TERMINAL);
}

//*****************************************************************************
//
//! Read a character from the console
//!
//! \param none
//!
//! \return Character
//
//*****************************************************************************
char getch(void)
{
    #define UART_CHAR_RECIEVED       0
    #define UART_CHAR_NOT_RECIEVED   (-1)

    uint8_t ch;
#if 0
    int8_t  ret = UART_CHAR_NOT_RECIEVED;
    while( ret == UART_CHAR_NOT_RECIEVED)
    {
        if (UARTCharAvailable(UARTLIN0_BASE))
        {
            // Return the character.
            ch = UARTGetCharNonBlocking(UARTLIN0_BASE);
            ret = UART_CHAR_RECIEVED;
        }
        else
        {
            ret = UART_CHAR_NOT_RECIEVED;
            osi_uSleep( 5 );
        }
    }
    return ch;
#endif
    size_t bytesRead;
    UART2_read(uartHandle, &ch, 1, &bytesRead);
    return ch;
} 

//*****************************************************************************
//
//! Outputs a character to the console
//!
//! \param[in]  char    - A character to be printed
//!
//! \return none
//
//*****************************************************************************
void putch(char ch)
{
    size_t bytesWritten;
    UART2_write(uartHandle, &ch, 1, &bytesWritten);
}
