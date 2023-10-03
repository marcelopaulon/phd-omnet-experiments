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
        @display("i=misc/sensor;p=220.74,7821.15");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=707.77,7957.40");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1071.36,7961.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1522.79,7990.95");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3387.90,7911.59");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3788.48,7777.92");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4604.23,7821.87");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1928.95,7726.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3031.44,7578.21");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4978.96,7636.28");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5653.38,7643.70");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=6.75,7341.20");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=411.62,7379.52");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=924.36,7333.82");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1429.99,7457.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3675.56,7312.27");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4042.57,7369.76");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5321.89,7497.90");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2224.97,7123.38");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4386.54,7244.47");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4929.53,7059.63");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=345.05,7007.41");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1292.39,6839.96");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2465.46,6857.47");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3262.88,6987.08");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3928.08,6997.30");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=4606.61,6869.64");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5292.45,6848.39");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5788.84,6986.53");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=448.36,6583.36");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=904.27,6636.31");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1569.65,6624.34");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2079.62,6547.26");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3212.41,6611.86");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3696.71,6563.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4167.09,6678.88");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4762.46,6537.81");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5729.28,6515.36");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=77.24,6467.61");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2647.27,6413.45");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=309.23,6122.84");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1232.87,6246.18");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2209.03,6174.09");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2904.44,6133.16");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4292.76,6079.34");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4778.39,6173.74");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5154.14,6232.04");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5552.05,6199.80");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5989.12,6098.08");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=805.16,5994.31");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=1911.19,5975.82");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=3958.14,5819.54");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=5322.45,5900.50");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=1299.18,5765.93");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=1657.04,5712.45");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=2160.10,5548.64");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=2550.80,5652.04");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=4840.97,5651.31");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=5576.35,5571.14");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=5886.67,5745.20");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=482.52,5461.35");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=935.01,5353.00");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=2475.36,5280.77");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=3071.66,5511.85");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=3521.42,5476.33");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=3996.85,5322.91");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=4347.43,5505.46");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=4924.04,5310.40");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5937.18,5303.79");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1489.70,5157.44");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=2072.05,5069.68");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=3567.85,5058.59");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=4382.31,5113.78");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=5302.17,5181.57");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=86.72,4961.47");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=542.37,4960.91");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1137.84,4899.34");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=1775.78,4813.51");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=2703.21,4954.35");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4005.37,4846.96");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=4613.73,4841.82");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=5054.97,4891.14");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=5487.08,4852.53");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=969.01,4558.81");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=2197.83,4638.90");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=3205.98,4769.79");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=5941.36,4647.67");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=529.01,4464.80");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1296.40,4410.19");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=1645.20,4455.40");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=2658.59,4470.74");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=3315.64,4422.78");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=3696.81,4511.80");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=5492.02,4355.35");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=89.87,4043.51");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=2230.41,4173.92");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=4436.01,4241.84");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4883.72,4179.39");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5881.77,4171.66");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=678.49,3943.62");
    }

}

