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

network DADCA_ACK_n100_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=184.01,132.37");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=31.62,208.00");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=100.10,272.05");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=156.73,236.68");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=130.77,161.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=31.66,286.22");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=96.84,348.20");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=168.81,393.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=29.80,451.51");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3.60,506.77");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=69.96,539.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=131.75,610.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=126.24,693.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=132.97,522.69");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=242.52,373.94");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=176.67,467.49");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=238.81,443.03");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=227.76,534.91");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=311.80,541.95");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=327.31,458.81");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=267.31,591.87");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=187.37,587.10");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=55.38,671.15");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=29.62,738.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=180.92,735.07");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=225.83,637.66");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=336.12,650.37");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=262.52,694.42");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=335.73,744.22");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=407.03,778.66");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=114.26,804.30");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=12.34,584.60");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=317.87,377.39");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=30.83,367.32");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=173.70,299.36");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=257.16,133.89");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=277.92,230.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=231.50,75.16");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=321.12,64.52");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=252.88,17.32");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=234.10,285.00");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=103.77,442.55");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=311.07,821.19");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=253.28,900.94");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=191.30,866.68");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=109.33,910.37");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=168.68,953.55");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=231.82,968.54");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=419.69,671.35");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=498.35,737.42");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=517.06,828.82");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=614.87,820.53");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=554.24,772.55");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=641.60,750.16");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=611.53,640.94");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=545.41,570.60");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=493.21,533.17");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=466.65,590.06");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=555.44,905.07");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=518.16,970.42");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=593.09,972.06");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=602.46,1046.10");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=617.78,896.23");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=449.33,904.45");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=384.35,870.21");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=332.79,968.37");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=288.81,1048.10");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=369.64,1045.96");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=443.03,1004.78");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=709.53,976.55");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=778.52,1003.06");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=697.31,1048.87");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=719.97,1144.70");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=620.49,1141.35");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=818.89,1106.44");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=870.09,1047.76");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=761.91,1075.14");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=719.77,906.54");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=740.91,791.85");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=817.33,774.01");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=891.83,840.30");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=809.83,835.78");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=848.07,887.02");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=892.95,759.04");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=947.75,689.96");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=1015.62,681.46");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=983.49,576.84");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=1045.61,509.51");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=962.59,500.87");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=986.73,423.99");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=1052.88,409.43");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=1139.25,468.92");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=1125.04,546.62");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=921.66,448.22");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=935.67,343.64");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=1061.61,569.26");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=878.23,265.41");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=851.95,328.93");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=760.83,336.14");
    }

}

