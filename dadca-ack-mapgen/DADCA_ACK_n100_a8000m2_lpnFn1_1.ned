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
        @display("i=misc/sensor,p=160.09,97.20");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=97.98,21.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=191.80,151.40");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=170.96,209.75");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=302.62,188.96");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=388.55,206.02");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=337.79,348.77");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=307.97,291.35");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=234.16,228.73");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=190.64,271.61");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=38.77,103.51");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=3.69,187.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=33.06,263.69");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=102.64,301.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=81.70,388.19");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=96.90,470.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=72.51,527.91");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=49.58,614.77");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=125.73,567.48");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=39.54,442.78");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=209.94,441.96");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=247.60,351.44");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=139.21,357.93");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=12.40,387.00");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=3.47,542.03");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=169.84,498.18");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=331.94,470.99");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=378.41,557.46");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=457.26,552.67");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=474.31,610.47");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=430.33,691.85");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=359.68,692.37");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=387.37,762.05");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=352.59,856.51");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=531.42,644.92");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=539.12,488.26");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=611.95,460.73");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=692.64,517.34");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=633.64,559.10");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=711.15,580.34");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=750.84,532.70");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=765.99,466.62");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=635.12,388.02");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=594.42,342.89");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=709.97,389.72");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=779.09,393.34");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=866.25,434.55");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=833.93,309.05");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=869.51,244.45");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=887.66,147.39");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=830.27,181.27");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=962.11,233.07");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=1032.89,289.74");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=1125.80,293.73");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=1178.20,211.55");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=1082.08,368.21");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=1247.83,225.43");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=1226.67,316.90");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=1304.99,363.83");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=1314.36,295.15");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=1139.20,447.45");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=1037.59,221.25");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=1067.15,114.19");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=998.42,64.15");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=1001.99,0.65");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=990.07,165.15");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=1121.23,177.53");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=954.07,321.45");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=1025.22,388.16");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=947.10,393.95");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=928.03,486.38");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=989.47,535.93");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=975.67,608.06");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=915.06,570.03");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=841.86,545.90");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=803.86,617.27");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=883.69,626.34");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=916.52,737.31");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=944.49,659.90");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=889.30,837.65");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=866.97,901.27");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=953.13,909.03");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=1023.17,940.84");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=1009.65,861.10");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=1019.93,776.35");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=824.90,763.65");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=759.87,705.50");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=728.76,756.88");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=675.46,700.58");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=590.54,766.71");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=558.69,842.29");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=539.04,728.39");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=516.23,786.31");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=589.59,919.21");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=547.39,1000.62");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=472.38,1032.80");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=382.06,1030.37");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=341.85,959.92");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=446.70,899.32");
    }

}

