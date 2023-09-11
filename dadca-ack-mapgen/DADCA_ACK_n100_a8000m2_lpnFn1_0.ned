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

network DADCA_ACK_n100_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=55.26,36.79");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=23.37,148.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=111.35,178.08");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=208.11,193.58");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=148.04,260.88");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=196.70,306.27");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=108.23,307.02");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=43.38,241.83");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=15.51,314.36");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=31.43,404.27");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=109.50,466.45");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=176.70,397.47");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=258.64,356.54");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=283.54,436.69");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=230.90,518.58");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=99.71,406.96");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=253.84,605.40");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=179.62,626.52");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=106.12,562.68");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=39.31,646.26");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=23.86,564.63");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=49.49,499.89");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=260.15,681.38");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=331.79,698.75");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=343.36,638.97");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=339.12,543.82");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=418.36,603.70");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=266.20,752.66");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=228.52,828.87");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=293.98,817.60");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=456.85,464.75");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=473.27,558.33");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=467.53,645.18");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=365.41,487.89");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=388.65,414.81");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=461.07,362.76");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=541.64,338.46");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=464.10,253.47");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=375.87,266.63");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=313.19,219.36");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=305.03,309.66");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=366.62,327.20");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=425.05,312.69");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=416.71,528.58");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=396.34,674.80");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=373.21,783.68");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=433.52,799.79");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=406.82,893.61");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=354.80,943.62");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=293.06,899.33");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=429.19,987.43");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=300.87,998.76");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=224.96,996.95");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=194.23,889.43");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=150.49,786.86");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=187.20,689.57");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=465.88,720.13");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=544.97,602.99");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=574.42,533.81");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=521.73,759.87");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=593.69,805.91");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=612.76,726.20");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=702.17,674.39");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=631.99,660.12");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=639.55,571.76");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=737.13,733.00");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=556.78,858.32");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=500.52,901.50");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=408.02,1065.17");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=524.31,1025.78");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=526.46,1087.24");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=481.47,1150.56");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=395.90,1129.29");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=386.60,1223.39");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=509.00,1246.33");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=447.58,1298.47");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=370.85,1327.54");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=441.32,1434.48");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=350.19,1451.94");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=420.40,1498.44");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=278.59,1519.26");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=254.82,1442.83");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=339.58,1549.68");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=361.04,1660.37");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=302.51,1711.34");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=258.52,1651.06");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=226.01,1598.96");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=163.74,1744.23");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=66.72,1723.18");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=161.04,1842.59");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=208.90,1887.96");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=225.56,1963.18");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=321.09,1949.14");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=294.63,2038.28");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=313.23,2133.32");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=349.21,2069.09");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=221.01,2145.48");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=307.63,2193.86");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=108.54,2250.69");
    }

}

