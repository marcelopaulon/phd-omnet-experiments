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

network DADCA_ACK_n100_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=46.80,54.82");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=38.34,132.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=14.26,199.49");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=212.81,91.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=170.18,25.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=339.86,73.55");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=300.29,19.19");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=393.06,15.28");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=455.26,59.59");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=221.07,153.79");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=67.94,238.46");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=144.43,218.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=81.75,174.71");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=87.77,305.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=16.86,279.11");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=172.39,383.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=150.24,470.26");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=235.78,426.25");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=250.75,357.05");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=348.35,476.97");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=308.30,527.01");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=337.43,401.17");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=399.39,335.08");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=372.28,281.53");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=380.10,212.35");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=334.95,152.33");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=53.13,415.74");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=60.47,495.63");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=165.86,535.70");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=98.82,566.21");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=233.01,596.15");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=258.99,679.63");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=309.66,646.75");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=213.03,283.38");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=309.11,273.48");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=240.73,508.39");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=384.21,666.05");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=333.29,744.94");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=365.29,825.26");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=428.51,870.14");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=357.85,914.85");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=380.26,1008.78");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=424.36,940.31");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=444.67,999.84");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=500.60,810.82");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=453.76,727.13");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=260.70,886.08");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=291.01,973.19");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=217.95,996.17");
    }

    sensors_51: MobileSensorNode {
        @display("i=misc/sensor,p=184.91,931.92");
    }

    sensors_52: MobileSensorNode {
        @display("i=misc/sensor,p=111.36,924.53");
    }

    sensors_53: MobileSensorNode {
        @display("i=misc/sensor,p=57.68,863.56");
    }

    sensors_54: MobileSensorNode {
        @display("i=misc/sensor,p=110.44,830.98");
    }

    sensors_55: MobileSensorNode {
        @display("i=misc/sensor,p=199.52,830.10");
    }

    sensors_56: MobileSensorNode {
        @display("i=misc/sensor,p=159.12,762.01");
    }

    sensors_57: MobileSensorNode {
        @display("i=misc/sensor,p=268.37,785.89");
    }

    sensors_58: MobileSensorNode {
        @display("i=misc/sensor,p=548.04,852.55");
    }

    sensors_59: MobileSensorNode {
        @display("i=misc/sensor,p=594.96,759.56");
    }

    sensors_60: MobileSensorNode {
        @display("i=misc/sensor,p=531.89,726.45");
    }

    sensors_61: MobileSensorNode {
        @display("i=misc/sensor,p=578.51,647.25");
    }

    sensors_62: MobileSensorNode {
        @display("i=misc/sensor,p=530.00,588.29");
    }

    sensors_63: MobileSensorNode {
        @display("i=misc/sensor,p=597.84,581.85");
    }

    sensors_64: MobileSensorNode {
        @display("i=misc/sensor,p=458.24,521.50");
    }

    sensors_65: MobileSensorNode {
        @display("i=misc/sensor,p=561.83,461.14");
    }

    sensors_66: MobileSensorNode {
        @display("i=misc/sensor,p=499.31,405.35");
    }

    sensors_67: MobileSensorNode {
        @display("i=misc/sensor,p=525.44,335.14");
    }

    sensors_68: MobileSensorNode {
        @display("i=misc/sensor,p=613.93,364.58");
    }

    sensors_69: MobileSensorNode {
        @display("i=misc/sensor,p=682.81,329.83");
    }

    sensors_70: MobileSensorNode {
        @display("i=misc/sensor,p=634.63,459.39");
    }

    sensors_71: MobileSensorNode {
        @display("i=misc/sensor,p=378.33,535.36");
    }

    sensors_72: MobileSensorNode {
        @display("i=misc/sensor,p=222.85,216.84");
    }

    sensors_73: MobileSensorNode {
        @display("i=misc/sensor,p=153.54,138.02");
    }

    sensors_74: MobileSensorNode {
        @display("i=misc/sensor,p=487.59,164.55");
    }

    sensors_75: MobileSensorNode {
        @display("i=misc/sensor,p=410.59,139.60");
    }

    sensors_76: MobileSensorNode {
        @display("i=misc/sensor,p=555.44,97.99");
    }

    sensors_77: MobileSensorNode {
        @display("i=misc/sensor,p=545.65,32.41");
    }

    sensors_78: MobileSensorNode {
        @display("i=misc/sensor,p=275.35,85.27");
    }

    sensors_79: MobileSensorNode {
        @display("i=misc/sensor,p=9.45,529.94");
    }

    sensors_80: MobileSensorNode {
        @display("i=misc/sensor,p=89.91,690.15");
    }

    sensors_81: MobileSensorNode {
        @display("i=misc/sensor,p=41.83,620.95");
    }

    sensors_82: MobileSensorNode {
        @display("i=misc/sensor,p=186.36,668.45");
    }

    sensors_83: MobileSensorNode {
        @display("i=misc/sensor,p=24.35,708.13");
    }

    sensors_84: MobileSensorNode {
        @display("i=misc/sensor,p=29.71,807.21");
    }

    sensors_85: MobileSensorNode {
        @display("i=misc/sensor,p=157.16,1002.37");
    }

    sensors_86: MobileSensorNode {
        @display("i=misc/sensor,p=95.10,1032.59");
    }

    sensors_87: MobileSensorNode {
        @display("i=misc/sensor,p=50.29,1093.42");
    }

    sensors_88: MobileSensorNode {
        @display("i=misc/sensor,p=214.22,731.36");
    }

    sensors_89: MobileSensorNode {
        @display("i=misc/sensor,p=441.13,207.23");
    }

    sensors_90: MobileSensorNode {
        @display("i=misc/sensor,p=497.63,247.79");
    }

    sensors_91: MobileSensorNode {
        @display("i=misc/sensor,p=620.15,37.65");
    }

    sensors_92: MobileSensorNode {
        @display("i=misc/sensor,p=583.01,185.08");
    }

    sensors_93: MobileSensorNode {
        @display("i=misc/sensor,p=716.55,29.72");
    }

    sensors_94: MobileSensorNode {
        @display("i=misc/sensor,p=660.99,104.79");
    }

    sensors_95: MobileSensorNode {
        @display("i=misc/sensor,p=812.36,98.08");
    }

    sensors_96: MobileSensorNode {
        @display("i=misc/sensor,p=792.11,178.84");
    }

    sensors_97: MobileSensorNode {
        @display("i=misc/sensor,p=794.49,278.27");
    }

    sensors_98: MobileSensorNode {
        @display("i=misc/sensor,p=670.80,176.46");
    }

    sensors_99: MobileSensorNode {
        @display("i=misc/sensor,p=657.21,254.58");
    }

    sensors_100: MobileSensorNode {
        @display("i=misc/sensor,p=148.88,306.98");
    }

}

