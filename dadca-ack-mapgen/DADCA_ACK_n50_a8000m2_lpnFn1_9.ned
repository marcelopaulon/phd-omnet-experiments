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

network DADCA_ACK_n50_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=122.78,197.03");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=36.05,189.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=69.50,261.84");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=15.92,306.63");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=36.23,381.08");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=114.58,397.72");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=127.85,336.30");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=148.74,265.07");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=210.32,408.94");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=304.10,332.63");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=72.63,457.58");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=45.22,531.16");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=115.76,515.05");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=129.04,595.64");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=49.04,601.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=114.88,654.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=188.97,616.82");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=201.97,556.13");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=261.54,572.91");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=340.21,606.39");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=278.05,644.64");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=203.15,491.24");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=153.26,451.41");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=9.29,68.44");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=223.73,77.88");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=227.05,165.14");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=285.61,227.90");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=238.55,306.17");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=333.11,385.88");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=387.26,456.07");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=308.16,496.33");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=454.70,373.51");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=221.82,677.63");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=301.53,715.41");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=313.49,786.57");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=258.38,869.98");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=200.65,910.89");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=359.43,746.42");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=244.78,739.99");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=378.86,809.56");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=446.98,835.18");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=522.44,779.75");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=573.58,838.43");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=658.54,766.08");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=632.92,688.33");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=568.68,710.83");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=529.62,625.05");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=538.38,530.57");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=448.45,630.11");
    }

}

