package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n100_u8_a8000m2_3
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=215.18,7983.32");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=634.85,7887.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2490.53,7859.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4064.98,7770.29");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4693.09,7872.75");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2900.06,7704.97");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3555.44,7744.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4982.49,7519.90");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5664.21,7710.68");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=227.70,7487.41");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=677.54,7473.68");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3072.64,7301.95");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3525.22,7364.37");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3904.20,7402.91");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4532.46,7475.55");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5457.44,7427.22");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5943.82,7332.70");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2431.90,7196.82");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3787.80,7057.75");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4294.79,7209.71");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5082.04,7126.45");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=576.71,6928.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1317.91,6918.84");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2031.66,6986.94");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3118.48,6801.51");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4732.83,6961.91");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5831.71,6861.91");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=873.19,6733.10");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2150.87,6587.57");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3492.33,6732.39");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=3821.84,6558.84");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=4687.29,6558.62");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5340.40,6679.07");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5702.30,6530.53");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1251.19,6351.63");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1635.80,6382.78");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2966.20,6474.67");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3302.65,6291.78");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4287.70,6465.42");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=447.10,6241.66");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1960.34,6151.30");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2494.27,6055.04");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3667.60,6211.90");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4888.19,6189.62");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5286.14,6045.77");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5949.62,6237.56");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1353.43,6001.37");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3118.03,5979.99");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3958.92,5923.41");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4612.87,5879.51");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=5872.89,5803.46");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=534.43,5528.55");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=857.73,5748.68");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1889.43,5761.17");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=2378.93,5551.72");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2737.52,5653.04");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3440.45,5666.35");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=5058.06,5768.83");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=9.25,5486.48");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1473.63,5454.04");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3175.33,5291.94");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3698.57,5422.60");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4267.81,5452.01");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5318.34,5375.15");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=5717.54,5388.95");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=768.61,5265.76");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=1151.52,5081.26");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=1969.86,5248.64");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=2358.69,5094.07");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=3554.45,5080.05");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=4922.13,5252.67");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=284.47,4948.82");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=1697.86,4899.33");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=2862.20,5014.82");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3930.04,4818.00");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4280.41,5008.98");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=4847.97,4841.47");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=5733.31,5015.22");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=762.59,4642.30");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=1188.95,4573.40");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=2035.98,4635.82");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=2568.06,4700.60");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=3155.44,4733.55");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=5393.72,4716.07");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=5899.64,4651.00");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=228.04,4355.38");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1575.48,4402.94");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=2384.23,4326.81");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2871.61,4401.53");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3677.05,4432.27");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4187.73,4455.84");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4609.10,4486.07");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5171.05,4377.13");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=3225.89,4228.76");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5646.84,4230.21");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=65.39,3991.28");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=645.26,3954.13");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1302.38,4001.24");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=1630.19,3806.39");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=2201.92,3804.75");
    }

}

