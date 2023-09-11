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

network DADCA_ACK_n20_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=112.69,160.52");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=131.08,225.60");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=89.24,302.15");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=32.17,283.06");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=26.87,211.17");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=13.57,387.22");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=114.52,394.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=38.99,444.06");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=197.04,283.85");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=191.54,399.39");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=277.08,352.34");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=255.48,255.22");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=265.18,185.25");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=337.34,174.19");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=280.03,126.53");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=349.87,63.71");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=285.48,64.57");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=187.32,74.16");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=228.32,11.71");
    }

}

