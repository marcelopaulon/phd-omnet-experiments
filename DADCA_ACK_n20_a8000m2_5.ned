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

network DADCA_ACK_n20_a8000m2_5
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
        @display("i=misc/sensor;p=1937.20,7792.64");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2592.21,7898.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3297.87,7948.30");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4705.05,7871.16");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1.00,7557.06");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1294.51,7718.54");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3659.45,7711.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4307.76,7674.77");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4815.65,7522.72");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=444.35,7504.48");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1066.55,7342.12");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2693.41,7419.01");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3061.28,7355.48");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3441.39,7425.31");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4508.66,7291.41");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5334.75,7414.95");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5798.39,7398.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=497.46,7083.50");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1546.31,7150.58");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2066.28,7150.59");
    }

}

