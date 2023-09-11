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

network DADCA_ACK_n20_a8000m2_lpnFn1_2
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
        @display("i=misc/sensor,p=27.41,45.67");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=81.66,0.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=166.61,43.11");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=150.56,159.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=79.61,157.00");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=33.34,224.89");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=110.14,254.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=147.45,319.29");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=137.02,394.79");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=228.26,380.48");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=44.69,389.70");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=68.58,333.95");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=254.16,252.74");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=234.91,192.48");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=215.64,102.23");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=297.21,103.68");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=321.56,162.65");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=410.73,77.28");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=525.35,10.08");
    }

}

