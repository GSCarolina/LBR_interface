package shared;

public enum Navigation {

	PLAY {
		@Override
		public TrainingTypeEnum getLevel() {
			return TrainingTypeEnum.ACTIVE;
		}
	},
	END {
		@Override
		public TrainingTypeEnum getLevel() {
			return TrainingTypeEnum.ACTIVE;
		}
	},
	RECORD {
		@Override
		public TrainingTypeEnum getLevel() {
			return TrainingTypeEnum.ACTIVE;
		}
	},
	NONE {
		@Override
		public TrainingTypeEnum getLevel() {
			return TrainingTypeEnum.ACTIVE;
		}
	};

	public abstract TrainingTypeEnum getLevel();
}
