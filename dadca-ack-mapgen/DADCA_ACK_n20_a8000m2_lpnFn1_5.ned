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

network DADCA_ACK_n20_a8000m2_lpnFn1_5
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
        @display("i=misc/sensor,p=13.40,137.07");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=43.26,227.48");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=107.23,179.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=68.68,306.98");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=156.93,255.68");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=161.65,350.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=126.91,413.58");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=256.69,271.18");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=326.47,337.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=313.41,241.18");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=325.01,160.13");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=263.06,73.59");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=213.80,196.50");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=155.28,134.80");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=171.90,459.76");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=220.83,416.52");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=291.38,452.53");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=271.84,514.66");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=246.78,570.19");
    }

}

