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

network DADCA_ACK_n50_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=81.16,171.09");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=25.18,116.53");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=168.86,44.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=252.01,46.83");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=326.56,119.88");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=397.91,136.70");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=417.86,75.86");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=509.25,80.92");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=196.79,166.51");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=236.88,213.41");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=291.46,285.47");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=307.22,365.03");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=339.33,40.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=98.73,11.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=38.80,257.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=29.78,353.46");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=279.46,163.34");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=333.48,189.79");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=400.99,216.75");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=473.56,242.55");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=532.94,168.57");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=563.21,238.86");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=558.68,312.18");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=653.64,267.48");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=666.08,206.57");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=715.63,277.46");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=487.16,301.20");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=635.81,331.05");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=565.20,376.59");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=499.37,381.19");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=596.31,158.32");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=459.12,141.80");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=677.47,75.04");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=606.01,54.76");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=417.47,373.44");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=438.88,461.48");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=396.84,526.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=350.31,610.58");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=335.93,444.88");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=270.66,456.73");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=228.69,533.86");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=197.51,324.49");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=163.98,270.79");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=88.50,306.38");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=92.37,408.32");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=144.68,366.24");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=141.01,453.39");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=48.78,452.58");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=193.20,483.40");
    }

}

