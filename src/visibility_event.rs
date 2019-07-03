#[derive(Debug, Clone, PartialEq)]
pub enum VisibilityEventType {
    StartVertex,
    EndVertex,
}

#[derive(Debug, Clone, PartialEq)]
pub struct VisibilityEvent {
    pub event_type: VisibilityEventType,
    pub segment: geo::Line<f64>,
}

impl VisibilityEvent {
    pub fn start(segment: &geo::Line<f64>) -> Self {
        Self {
            event_type: VisibilityEventType::StartVertex,
            segment: *segment,
        }
    }

    pub fn end(segment: &geo::Line<f64>) -> Self {
        Self {
            event_type: VisibilityEventType::EndVertex,
            segment: *segment,
        }
    }

    pub fn point(&self) -> geo::Point<f64> {
        self.segment.start.into()
    }
}
