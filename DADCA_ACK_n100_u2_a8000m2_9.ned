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

network DADCA_ACK_n100_u2_a8000m2_9
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
        @display("i=misc/sensor;p=1242.96,7841.29");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2297.79,7991.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2924.54,7934.92");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3305.53,7822.62");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4456.21,7833.22");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5309.93,7806.51");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=289.41,7600.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3842.20,7583.28");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4910.02,7528.95");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5749.15,7670.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=436.65,7271.10");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=778.43,7493.55");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1167.92,7293.23");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1637.25,7389.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2271.45,7409.65");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2760.95,7481.68");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3396.49,7397.00");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4280.25,7310.73");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=23.61,7036.16");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1943.92,7153.98");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3216.21,7066.30");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3817.24,7155.67");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5483.33,7230.13");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5929.14,7078.40");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=668.67,6922.64");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1030.25,6945.99");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1398.17,6938.78");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2550.01,6868.12");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4633.21,6915.61");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5346.17,6882.57");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5740.26,6774.97");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=226.29,6723.17");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1896.78,6725.11");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2212.85,6562.43");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2978.80,6756.10");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3567.74,6515.95");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3975.39,6696.32");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=41.41,6378.53");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=594.26,6425.43");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1541.13,6320.73");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2626.34,6496.59");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3111.40,6402.95");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4578.88,6369.38");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4976.20,6468.53");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5330.82,6293.55");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5753.26,6322.31");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=578.92,6061.61");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1112.77,6260.64");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1979.93,6254.06");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4015.33,6203.27");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=253.17,5836.53");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=1589.94,5943.95");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=2215.86,5916.09");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=2620.12,5973.15");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=3143.75,5938.74");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=4907.63,5832.64");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=5474.14,5959.27");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=787.30,5579.13");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=1183.63,5741.99");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=1670.17,5579.52");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=2154.42,5553.08");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=2720.23,5549.56");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=3437.50,5739.05");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=4038.70,5735.47");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=4398.29,5692.97");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=5348.97,5608.82");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=5976.00,5570.08");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=284.11,5413.01");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=293.81,5059.21");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=1003.51,5185.04");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=1656.58,5159.43");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=2296.97,5153.73");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=3138.30,5267.00");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=3993.47,5128.07");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=5051.08,5163.76");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=1359.74,4929.38");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=1911.98,4830.90");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=3532.93,5012.60");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=4374.05,4852.06");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=4891.79,4822.07");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=5630.40,4889.37");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=879.13,4736.14");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=2559.73,4732.26");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=3191.77,4739.14");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=3955.78,4614.00");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=5239.92,4667.60");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=272.82,4408.83");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=1444.12,4470.31");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=2327.41,4391.23");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=3350.70,4423.36");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=4791.40,4320.54");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=5639.31,4441.36");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=50.11,4048.71");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=472.31,4118.91");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=986.74,4259.41");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=1999.34,4044.84");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=3191.57,4062.74");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=4131.92,4059.69");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=5238.92,4158.76");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=1135.41,3928.67");
    }

}

