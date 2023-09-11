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
        @display("i=misc/sensor,p=163.31,88.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=135.42,9.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=193.56,30.57");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=289.20,14.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=256.71,122.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=252.91,184.22");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=322.42,222.79");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=369.55,168.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=140.08,215.29");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=119.66,292.13");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=175.24,323.07");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=155.12,412.66");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=82.53,347.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=33.25,407.54");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=86.18,477.17");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=152.75,501.93");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=160.28,599.69");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=113.35,671.51");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=59.32,719.59");
    }

}

