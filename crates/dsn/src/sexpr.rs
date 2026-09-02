//! Tiny s-expression reader for Specctra DSN/SES files.

#[derive(Debug, Clone, PartialEq)]
pub enum SExpr {
    Atom(String),
    List(Vec<SExpr>),
}

impl SExpr {
    /// First atom of a list (the "name" of the form), if any.
    pub fn name(&self) -> Option<&str> {
        match self {
            SExpr::List(items) => match items.first() {
                Some(SExpr::Atom(s)) => Some(s.as_str()),
                _ => None,
            },
            _ => None,
        }
    }

    pub fn as_atom(&self) -> Option<&str> {
        match self {
            SExpr::Atom(s) => Some(s.as_str()),
            _ => None,
        }
    }

    pub fn items(&self) -> &[SExpr] {
        match self {
            SExpr::List(items) => items,
            _ => &[],
        }
    }

    /// Children after the name atom.
    pub fn args(&self) -> &[SExpr] {
        let items = self.items();
        if items.is_empty() { items } else { &items[1..] }
    }

    /// All child lists named `name`.
    pub fn children<'a>(&'a self, name: &'a str) -> impl Iterator<Item = &'a SExpr> + 'a {
        self.items().iter().filter(move |e| e.name() == Some(name))
    }

    pub fn child<'a>(&'a self, name: &str) -> Option<&'a SExpr> {
        self.items().iter().find(|e| e.name() == Some(name))
    }

    /// Positional argument as f64.
    pub fn f64_arg(&self, i: usize) -> Option<f64> {
        self.args().get(i)?.as_atom()?.parse().ok()
    }

    pub fn str_arg(&self, i: usize) -> Option<&str> {
        self.args().get(i)?.as_atom()
    }
}

/// Parse a whole DSN/SES document. Returns the single top-level form.
pub fn parse(text: &str) -> Result<SExpr, String> {
    // KiCad emits `(string_quote ")` inside the parser section: a lone quote
    // atom that breaks naive string scanning. Neutralize it before tokenizing.
    let text = text
        .replace("(string_quote \")", "(string_quote QUOTE)")
        .replace("(string_quote ')", "(string_quote APOS)");
    let mut tokens = tokenize(&text)?;
    tokens.reverse(); // pop from the end
    let expr = parse_expr(&mut tokens)?;
    Ok(expr)
}

#[derive(Debug)]
enum Token {
    Open,
    Close,
    Atom(String),
}

fn tokenize(text: &str) -> Result<Vec<Token>, String> {
    let mut out = Vec::new();
    let bytes = text.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        let c = bytes[i] as char;
        match c {
            '(' => {
                out.push(Token::Open);
                i += 1;
            }
            ')' => {
                out.push(Token::Close);
                i += 1;
            }
            '"' => {
                let start = i + 1;
                let mut j = start;
                while j < bytes.len() && bytes[j] as char != '"' {
                    j += 1;
                }
                if j >= bytes.len() {
                    return Err("unterminated string".into());
                }
                out.push(Token::Atom(text[start..j].to_string()));
                i = j + 1;
            }
            c if c.is_whitespace() => {
                i += 1;
            }
            '#' => {
                // comment to end of line
                while i < bytes.len() && bytes[i] as char != '\n' {
                    i += 1;
                }
            }
            _ => {
                let start = i;
                while i < bytes.len() {
                    let c = bytes[i] as char;
                    if c.is_whitespace() || c == '(' || c == ')' || c == '"' {
                        break;
                    }
                    i += 1;
                }
                out.push(Token::Atom(text[start..i].to_string()));
            }
        }
    }
    Ok(out)
}

fn parse_expr(tokens: &mut Vec<Token>) -> Result<SExpr, String> {
    match tokens.pop() {
        Some(Token::Open) => {
            let mut items = Vec::new();
            loop {
                match tokens.last() {
                    Some(Token::Close) => {
                        tokens.pop();
                        return Ok(SExpr::List(items));
                    }
                    None => return Err("unbalanced parentheses".into()),
                    _ => items.push(parse_expr(tokens)?),
                }
            }
        }
        Some(Token::Atom(s)) => Ok(SExpr::Atom(s)),
        Some(Token::Close) => Err("unexpected ')'".into()),
        None => Err("empty input".into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_basic() {
        let e = parse("(pcb \"my board\" (layer F.Cu (type signal)))").unwrap();
        assert_eq!(e.name(), Some("pcb"));
        assert_eq!(e.str_arg(0), Some("my board"));
        let layer = e.child("layer").unwrap();
        assert_eq!(layer.str_arg(0), Some("F.Cu"));
        assert_eq!(layer.child("type").unwrap().str_arg(0), Some("signal"));
    }

    #[test]
    fn test_string_quote_hack() {
        let e = parse("(parser (string_quote \") (space_in_quoted_tokens on))").unwrap();
        assert_eq!(e.name(), Some("parser"));
        assert!(e.child("space_in_quoted_tokens").is_some());
    }
}
