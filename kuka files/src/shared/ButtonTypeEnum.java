package shared;


public enum ButtonTypeEnum {

	PLAY {
		@Override
		public TrainingTypeEnum getLevel() {
			return levelValue;
		}

		@Override
		public void setLevel(TrainingTypeEnum level) {
			levelValue = level;
			
		}
	},
	END {
		@Override
		public TrainingTypeEnum getLevel() {
			return levelValue;
		}

		@Override
		public void setLevel(TrainingTypeEnum level) {
			levelValue = level;
			
		}
	},
	RECORD {
		@Override
		public TrainingTypeEnum getLevel() {
			return levelValue;
		}

		@Override
		public void setLevel(TrainingTypeEnum level) {
			levelValue = level;			
		}
	},
	NONE {
		@Override
		public TrainingTypeEnum getLevel() {
			return levelValue;
		}

		@Override
		public void setLevel(TrainingTypeEnum level) {
			levelValue = level;
			
		}
	};

	protected TrainingTypeEnum levelValue;
	
//	Navi() {
//		// creating dictionary
//		trainingNavi = new EnumMap<Navi,  TrainingTypeEnum>(Navi.class);
//		buttonClicked = new EnumMap<Navi, Boolean>(Navi.class);

		//resetNavi();
//	}

	public abstract TrainingTypeEnum getLevel();
	public abstract void setLevel(TrainingTypeEnum level);

	
}
