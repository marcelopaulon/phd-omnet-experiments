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

network DADCA_ACK_n100_u4_a8000m2_1
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
        @display("i=misc/sensor;p=1054.66,7962.25");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1488.49,7979.53");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2476.85,7760.86");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2908.68,7785.00");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3452.70,7999.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4559.29,7992.46");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5328.69,7958.58");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5800.27,7847.04");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=237.99,7554.97");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=745.21,7662.81");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1156.57,7605.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1533.33,7588.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2068.26,7620.15");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3937.73,7647.67");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4981.34,7610.26");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5434.72,7576.81");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=638.60,7311.66");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=993.25,7274.19");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2785.42,7347.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3175.47,7456.83");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3755.96,7257.59");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4137.85,7258.57");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4530.46,7399.08");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5744.57,7258.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=1429.85,7189.11");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1819.02,7209.53");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4851.54,7063.39");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5410.21,7084.95");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=64.23,6987.13");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1237.63,6770.98");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2389.81,6994.12");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3738.98,6877.67");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4430.21,6816.57");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=394.46,6760.54");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=864.20,6751.44");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1757.40,6699.48");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=2725.92,6660.15");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4173.75,6555.56");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5150.57,6567.33");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5740.44,6596.64");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=672.45,6299.68");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1870.58,6346.05");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2290.05,6347.86");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3127.58,6453.47");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3575.96,6500.61");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4663.35,6408.91");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5462.24,6358.80");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3439.46,6109.28");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3990.41,6228.12");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5891.06,6171.76");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1107.41,5975.54");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=2673.49,5984.28");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=3957.28,5837.07");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=4647.33,5988.25");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=5006.97,5977.89");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=5658.17,5796.82");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=69.57,5552.09");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=426.69,5650.22");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1962.10,5753.11");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=2966.38,5755.77");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=5065.44,5553.85");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=5993.42,5566.72");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=1316.51,5322.72");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=2500.15,5456.84");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3611.33,5456.31");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=4245.80,5427.08");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4591.22,5320.03");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=5555.97,5328.33");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=629.48,5245.77");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=980.08,5175.01");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1874.90,5270.67");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3226.95,5224.94");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3900.07,5226.32");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=4961.21,5165.84");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5898.45,5172.97");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1470.20,4965.33");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=2979.46,4972.67");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3654.72,4970.95");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=3955.58,4789.49");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4504.75,4890.40");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5371.28,4783.67");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=368.87,4766.97");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=907.96,4591.59");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=1248.31,4679.97");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1607.11,4564.67");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1960.53,4626.03");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=2398.99,4674.71");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=3119.93,4596.56");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=3498.61,4580.75");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=4867.66,4768.77");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4315.40,4434.38");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=4834.10,4335.87");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=5726.86,4303.94");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=180.79,4177.61");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1310.94,4208.72");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2050.02,4243.48");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2786.51,4125.02");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3914.34,4280.96");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5291.49,4086.02");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=403.71,3891.81");
    }

}

