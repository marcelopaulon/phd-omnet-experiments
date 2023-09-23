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

network DADCA_ACK_n100_u4_a8000m2_7
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
        @display("i=misc/sensor;p=295.03,7909.46");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=858.70,7849.86");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1983.10,7820.17");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2332.75,7886.04");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4953.93,7843.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5599.00,7881.30");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2726.01,7674.26");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3235.79,7697.81");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4118.05,7621.36");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4495.36,7643.28");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=29.94,7497.94");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=553.19,7321.61");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1369.90,7503.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2097.20,7406.33");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2599.90,7273.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2996.55,7428.11");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4683.24,7326.42");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5105.40,7318.41");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5677.02,7417.66");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=917.81,7214.07");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1229.11,7053.15");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3610.34,7109.98");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4050.20,7219.76");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5888.92,7079.93");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=74.84,6929.24");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1908.51,6980.50");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2702.12,6822.03");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4318.71,6796.96");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4790.92,6987.41");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=521.77,6750.25");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=848.41,6528.06");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1794.52,6586.98");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2347.16,6718.40");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3207.05,6638.60");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3767.74,6678.25");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5726.18,6726.47");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=213.36,6274.79");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1220.75,6375.34");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4120.10,6301.85");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=4717.72,6484.35");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=5256.45,6501.97");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5894.09,6311.12");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1584.37,6210.58");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2216.28,6183.35");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2951.86,6166.41");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3598.59,6254.23");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4457.63,6117.29");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4914.34,6044.96");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5477.58,6077.79");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=538.16,6015.22");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=953.65,6015.05");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1517.00,5809.05");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=1892.58,5970.50");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2473.73,5803.95");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3472.85,5902.76");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4049.87,5941.00");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=50.38,5661.66");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=1134.74,5606.08");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=2037.03,5532.97");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=3247.18,5536.36");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=3767.94,5702.25");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=4539.54,5555.17");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=5309.65,5742.22");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=5869.66,5747.14");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=651.01,5510.48");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=1428.62,5284.00");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=2578.57,5456.49");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4939.26,5420.37");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5529.34,5452.16");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=5989.04,5410.92");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1747.31,5100.88");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2229.25,5139.81");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=2914.61,5228.19");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3442.15,5121.89");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=3804.97,5145.02");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=4445.07,5160.65");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=5222.98,5188.10");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=390.72,4993.15");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=974.17,4997.28");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=2662.46,4808.27");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=3268.99,4798.75");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5001.16,4890.06");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5799.01,4867.09");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=436.96,4633.05");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=1302.13,4624.26");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=1960.76,4718.37");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=4019.58,4612.94");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=4361.93,4746.51");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=4696.85,4595.35");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=5488.83,4606.20");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=791.96,4434.50");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=2796.03,4448.95");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=4213.08,4320.80");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5930.23,4404.53");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=1090.55,4078.51");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1482.65,4277.41");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=2454.66,4222.82");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=3286.98,4284.44");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=3858.09,4046.76");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=4813.55,4064.37");
    }

}

