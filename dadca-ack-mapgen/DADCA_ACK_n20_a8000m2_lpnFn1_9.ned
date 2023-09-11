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

network DADCA_ACK_n20_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=160.51,175.56");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=136.56,33.67");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=28.70,126.02");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=12.26,193.11");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=42.54,250.39");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=129.89,265.09");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=94.23,336.77");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=280.14,239.16");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=192.86,273.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=263.69,153.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=245.07,58.16");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=296.07,99.20");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=210.89,111.79");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=359.61,122.57");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=430.44,180.80");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=453.15,267.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=455.17,62.87");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=541.49,14.82");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=628.32,1.22");
    }

}

