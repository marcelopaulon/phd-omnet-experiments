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

network DADCA_ACK_n20_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=21.22,84.60");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=75.39,1.85");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=96.41,164.94");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=11.07,148.37");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=192.87,74.18");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=212.56,133.85");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=168.39,210.74");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=256.42,209.84");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=279.52,302.90");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=215.77,299.17");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=113.84,302.26");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=53.37,236.96");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=75.93,391.06");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=154.54,444.11");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=162.15,514.23");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=162.92,355.60");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=260.12,6.66");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=358.90,11.76");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=367.77,84.40");
    }

}

