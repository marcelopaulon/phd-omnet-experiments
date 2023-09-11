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

network DADCA_ACK_n100_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor;p=157.59,122.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=130.11,212.27");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=146.44,281.01");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=81.15,157.71");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=27.91,235.73");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=22.02,308.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=43.77,390.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=119.45,365.58");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=188.30,407.07");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=286.98,416.91");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=235.34,332.67");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=349.18,296.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=377.45,371.53");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=234.92,479.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=283.18,516.84");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=356.67,477.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=439.56,453.97");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=524.46,435.98");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=510.08,515.25");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=577.37,578.91");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=652.64,615.55");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=574.94,658.81");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=521.22,716.26");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=527.49,806.98");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=680.63,675.67");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=690.36,541.61");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=673.51,452.03");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=701.64,360.42");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=628.67,387.52");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=778.19,400.04");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=837.37,366.11");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=917.02,411.13");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=906.52,508.15");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=929.87,591.91");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=870.37,610.55");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=822.87,650.65");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=912.93,685.55");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=893.47,744.57");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=817.38,728.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=758.73,680.43");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=714.83,733.43");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=632.50,784.74");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=670.45,860.88");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=584.73,868.38");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=454.27,637.59");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=400.99,582.56");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=333.04,560.36");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=364.99,674.76");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=465.69,757.22");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor;p=439.93,812.55");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor;p=371.42,869.76");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor;p=351.38,793.01");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor;p=413.11,722.41");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor;p=326.27,721.39");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor;p=255.06,685.71");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor;p=184.76,726.82");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor;p=141.97,640.92");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor;p=154.91,781.52");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor;p=204.44,846.78");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor;p=215.80,544.79");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor;p=224.80,776.01");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor;p=311.58,632.88");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor;p=243.28,607.98");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor;p=165.90,582.94");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor;p=123.77,523.45");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor;p=62.94,567.65");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor;p=34.64,838.67");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor;p=5.44,782.69");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor;p=57.62,910.21");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor;p=85.22,992.87");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor;p=10.83,967.91");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor;p=142.41,860.33");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor;p=190.19,920.54");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor;p=194.86,1020.39");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor;p=266.67,1007.37");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor;p=321.88,957.00");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor;p=283.64,879.80");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor;p=83.73,670.17");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor;p=12.06,617.35");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor;p=32.81,492.84");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor;p=110.04,438.23");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor;p=273.47,243.78");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor;p=204.70,190.05");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor;p=290.80,181.55");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor;p=379.14,243.89");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor;p=132.08,44.27");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor;p=220.33,28.06");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor;p=235.49,100.24");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor;p=367.86,147.66");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor;p=386.69,62.14");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor;p=461.79,195.00");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor;p=502.73,586.20");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor;p=412.89,518.50");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor;p=496.16,951.15");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor;p=442.97,910.16");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor;p=289.15,778.13");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor;p=591.09,492.16");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor;p=481.38,346.27");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor;p=535.90,376.73");
    }

}

