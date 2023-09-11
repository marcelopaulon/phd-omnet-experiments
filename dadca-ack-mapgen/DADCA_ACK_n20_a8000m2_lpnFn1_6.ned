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

network DADCA_ACK_n20_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor,p=178.97,49.45");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=236.38,121.90");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=271.25,39.85");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=341.99,109.29");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=373.85,48.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=507.93,49.20");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=449.54,75.33");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=454.33,4.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=486.73,132.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=402.01,162.31");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=462.49,199.42");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=467.87,271.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=538.21,218.99");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=558.54,90.58");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=588.60,149.11");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=667.51,121.00");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=766.63,108.86");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=853.32,130.69");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=916.32,174.68");
    }

}

