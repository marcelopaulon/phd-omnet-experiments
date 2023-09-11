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

network DADCA_ACK_n20_a8000m2_lpnFn1_0
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
        @display("i=misc/sensor,p=109.88,39.33");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=30.61,93.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=175.81,37.58");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=183.85,134.51");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=116.72,173.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=88.61,239.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=26.48,312.84");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=148.66,349.68");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=218.59,300.20");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=300.95,337.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=347.88,261.73");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=302.60,204.71");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=266.12,262.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=44.02,160.67");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=156.46,249.11");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=125.84,465.88");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=38.89,472.31");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=10.22,396.38");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=42.39,534.93");
    }

}

