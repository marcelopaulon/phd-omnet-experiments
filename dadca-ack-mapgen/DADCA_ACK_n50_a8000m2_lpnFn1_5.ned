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

network DADCA_ACK_n50_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=190.28,91.52");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=269.87,73.34");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=321.44,117.51");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=360.54,50.41");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=185.37,15.66");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=384.69,163.45");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=431.06,212.13");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=510.01,154.23");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=418.73,292.59");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=363.05,350.94");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=314.92,233.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=485.52,246.83");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=536.27,289.67");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=614.45,302.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=587.05,207.68");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=439.34,56.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=591.46,102.30");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=433.25,118.32");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=512.29,63.69");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=250.84,223.73");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=279.75,351.80");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=302.43,408.47");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=264.17,474.11");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=230.04,561.35");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=203.16,488.19");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=125.29,520.86");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=145.60,597.15");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=193.88,280.08");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=151.15,220.34");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=85.76,215.15");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=24.19,155.60");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=27.01,90.36");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=66.10,18.42");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=140.86,157.53");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=185.39,354.32");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=309.35,540.01");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=266.80,627.75");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=332.26,692.54");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=204.38,662.93");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=39.23,601.73");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=81.70,663.98");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=151.35,732.49");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=144.08,661.81");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=376.55,461.20");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=437.35,536.68");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=502.48,525.96");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=552.06,603.66");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=458.68,658.37");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=480.79,748.99");
    }

}

