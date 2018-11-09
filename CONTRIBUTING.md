    
## Documenting the code

FABlin code is documented following NaturalDocs syntax. This tool permit to write
both readable and efficient comments for documenting the code, and is very adaptable
to one's needs.

### Documenting g-code commands

Marlin_main.cpp contains block comments for documenting g-code commands. It is
important to expand these comments in order to have a nicely readable user and API
documentation.

Elements of command docs comment:

    /**
     * Command: COMMAND name
     * 
     *  BRIEF description for the command, no full stops here
     *
     * --- Prototype ---
     * COMMAND syntax
     * -----------------
     *
     * Parameters:
     *  LETTER - Brief explanation; value range; default value
     *
     * Description:
     *  A MORE lengthy description goes here. Just describe less-than obvious features.
     * Remember full stops here. 
     * 
     * See Also:
     *  LINKS to companion commands
     * 
     * Compatibility:
     *  STATE copmatibility with other firmwares, especially if not Marlin. For FABlin
     * custom command, use FABlin
     */

### Documenting the API

When documenting the rest of the code, the most important part are the public API of modules, whereas
internal methods can be skipped.
