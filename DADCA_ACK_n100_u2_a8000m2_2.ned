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

network DADCA_ACK_n100_u2_a8000m2_2
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
        @display("i=misc/sensor;p=2481.34,7851.54");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4877.66,7923.06");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5547.68,7955.84");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=352.61,7712.91");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=809.39,7668.30");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1216.13,7637.48");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1869.02,7577.78");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3750.84,7607.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4359.24,7730.88");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5093.06,7630.43");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5874.71,7703.99");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=211.11,7343.96");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=671.03,7307.46");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2230.30,7435.93");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3121.92,7347.49");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4496.75,7385.15");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5038.36,7274.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1078.35,7130.66");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1704.67,7254.20");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2353.44,7012.97");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2770.38,7124.52");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3480.79,7235.80");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3992.24,7214.22");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4742.24,7085.44");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5559.19,7092.38");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=285.97,6899.18");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1468.75,6995.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3094.28,6784.82");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4427.99,6907.97");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5279.11,6811.19");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=444.70,6547.00");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=815.65,6728.47");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1426.23,6612.93");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1798.14,6610.79");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2490.92,6567.42");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3648.62,6610.27");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5952.13,6523.71");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=3237.25,6427.22");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4276.56,6428.86");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4907.75,6482.26");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5544.42,6477.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=219.69,6167.68");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=828.62,6083.03");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2115.38,6190.96");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2614.35,6236.06");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3055.34,6113.48");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4139.70,6060.14");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5257.33,6177.81");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1722.91,5956.60");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2362.77,5809.30");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=2792.88,5816.86");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3517.61,6004.04");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=4694.24,5888.57");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=5703.08,5904.53");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=438.30,5749.10");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=1429.80,5531.45");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=3114.61,5658.16");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=3937.82,5766.41");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=4272.94,5629.52");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5127.96,5601.03");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5998.33,5622.47");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=28.62,5504.77");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=983.97,5422.53");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=1728.38,5298.18");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=2140.51,5521.39");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3481.76,5342.63");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4757.16,5512.08");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5625.31,5330.99");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=1383.28,5120.80");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=2392.82,5037.46");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2673.46,5269.39");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3879.01,5043.79");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=5162.34,5081.40");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5859.21,5069.54");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=596.13,4891.31");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1187.64,4787.43");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1814.75,4952.62");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3043.58,4996.75");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3588.70,4840.19");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4483.79,5006.53");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4843.52,4853.63");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5502.65,4979.48");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=1765.68,4590.15");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=2705.67,4628.38");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=4206.32,4712.92");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5267.45,4614.15");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5945.82,4619.65");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=769.41,4424.61");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1417.48,4300.67");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=2358.85,4450.80");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=3835.83,4369.89");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4501.25,4392.07");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4860.94,4468.60");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5513.32,4301.50");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=442.35,4089.75");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1074.98,4115.66");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2667.66,4216.61");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4219.05,4161.35");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5115.24,4182.43");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=853.38,3824.58");
    }

}

