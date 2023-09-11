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

network DADCA_ACK_n100_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=133.03,157.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=201.53,201.78");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=238.22,137.59");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=279.20,206.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=373.75,217.47");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=340.18,164.40");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=226.60,310.14");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=177.01,394.53");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=242.21,426.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=276.35,480.98");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=153.32,316.28");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=77.13,262.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=77.93,434.17");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=107.80,532.57");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=138.32,618.04");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=217.43,624.95");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=277.59,586.22");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=363.19,534.85");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=344.83,599.07");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=413.44,467.93");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=400.71,399.93");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=432.87,348.86");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=444.83,525.41");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=492.71,305.52");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=422.62,281.66");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=478.71,423.08");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=550.19,425.05");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=637.66,433.45");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=704.47,486.03");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=688.50,556.23");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=725.88,648.87");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=803.30,593.74");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=785.19,660.86");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=844.30,535.30");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=840.32,446.48");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=860.05,358.92");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=885.37,302.11");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=840.78,216.40");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=752.03,197.09");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=795.91,152.81");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=952.89,343.27");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1016.86,290.26");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1104.22,282.88");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1132.97,200.89");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1066.64,207.57");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=964.22,225.94");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=787.06,309.91");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=779.82,441.82");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=973.44,455.95");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=961.21,535.36");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=901.20,513.69");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=1054.02,393.97");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=1104.53,356.91");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=1231.26,338.56");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1317.60,302.96");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1376.93,344.80");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1451.98,323.48");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1502.94,364.72");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1573.51,327.46");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1556.87,399.15");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1424.17,431.75");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1397.84,280.08");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1416.64,194.51");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=1479.72,243.93");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1552.59,257.13");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=1591.03,203.96");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1670.87,199.93");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=1614.13,277.36");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1697.13,283.67");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=1784.64,325.70");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=1847.43,346.83");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=1816.96,416.44");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=1870.70,479.76");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=1809.99,482.34");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=1853.42,538.36");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=1899.16,583.40");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=1954.53,523.18");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=2027.29,475.69");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=1944.53,428.85");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=1852.80,265.46");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=1931.38,279.51");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=1821.90,200.35");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1884.84,139.88");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1801.47,131.29");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1831.98,44.11");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=1952.30,107.97");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=1947.94,10.02");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=1886.52,15.91");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=2079.62,58.05");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=2149.82,14.94");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=2096.01,179.54");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=2005.06,212.01");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=1942.30,200.63");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=2123.36,285.53");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=2116.99,381.16");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=2206.76,412.39");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=2270.51,417.36");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=2233.75,501.44");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=2168.95,494.19");
    }

}

