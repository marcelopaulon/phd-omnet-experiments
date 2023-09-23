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

network DADCA_ACK_n20_u4_a8000m2_9
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
        @display("i=misc/sensor;p=1368.82,7989.91");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=4496.53,7930.80");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=5112.22,7820.40");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5470.15,7792.66");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5832.85,7952.15");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=701.99,7563.93");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1095.24,7641.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3193.48,7598.72");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4766.59,7513.59");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1590.64,7460.29");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2203.15,7441.95");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2726.09,7419.76");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4395.35,7472.92");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5325.64,7441.60");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5758.44,7493.28");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=188.78,7207.85");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=749.19,7187.81");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1887.40,7089.67");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2365.03,7070.65");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2837.73,7033.79");
    }

}

