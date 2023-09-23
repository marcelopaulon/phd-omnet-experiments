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

network DADCA_ACK_n100_u2_a8000m2_0
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
        @display("i=misc/sensor;p=692.83,7865.24");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1341.58,7857.20");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2759.34,7752.86");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3847.30,7778.41");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4296.86,7837.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4757.40,7974.65");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5758.99,7907.93");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=65.41,7506.20");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1696.51,7707.86");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=620.07,7387.24");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1181.17,7308.49");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2223.00,7419.69");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2957.17,7259.23");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3305.51,7441.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3667.79,7396.97");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4286.71,7313.08");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5104.94,7423.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5578.36,7489.39");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1747.62,7153.40");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5561.20,7106.35");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=335.62,7004.80");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=801.90,6849.68");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1156.01,6831.95");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1631.22,6803.64");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2600.25,6917.10");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3024.95,6822.12");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3663.18,6936.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4062.93,6915.88");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4494.42,6838.56");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4816.41,6995.25");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=537.55,6611.72");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2726.24,6544.38");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3269.46,6560.76");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3635.80,6538.16");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4231.87,6583.71");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5182.30,6719.58");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5943.68,6679.62");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=793.13,6318.29");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1323.98,6472.56");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2092.82,6453.86");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4612.57,6408.41");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5420.91,6292.20");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=368.08,6258.96");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1670.67,6174.29");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3288.23,6032.45");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3729.64,6091.22");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4159.80,6040.85");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4805.06,6036.24");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5801.87,6171.11");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=929.56,5904.47");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1319.47,5986.90");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2066.28,5785.69");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2579.13,5905.90");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5249.75,5797.11");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=20.23,5764.48");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=3069.10,5686.50");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3648.67,5673.00");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4087.83,5669.45");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4530.03,5768.83");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5666.97,5701.99");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=1478.28,5478.21");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=3279.81,5283.08");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=4707.55,5402.99");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=149.96,5148.61");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=615.13,5187.56");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1751.47,5214.05");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2405.27,5230.15");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=2826.94,5040.43");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=4134.73,5128.90");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5197.63,5059.98");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=350.47,4860.56");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=1155.44,4906.79");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2245.19,4808.84");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3342.79,4902.29");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3826.30,4794.41");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4610.63,4903.88");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2.97,4604.34");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=801.29,4659.96");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=1739.36,4774.32");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2635.31,4659.47");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3175.32,4587.69");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5073.47,4694.67");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5588.93,4615.43");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=385.47,4490.72");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1449.16,4418.70");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3478.79,4322.99");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4102.31,4468.26");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4491.64,4427.68");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1104.09,4149.01");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2131.24,4285.24");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2805.97,4224.32");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4217.26,4069.71");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4771.17,4047.25");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5297.45,4255.43");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=5842.60,4104.11");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=253.80,4004.96");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=757.98,3927.48");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=1734.93,3839.62");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=2604.75,3870.11");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=3164.31,3850.18");
    }

}

