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

network DADCA_ACK_n20_a8000m2_2
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
        @display("i=misc/sensor;p=200.36,7984.86");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=303.55,7971.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=620.35,7989.15");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=970.62,7990.54");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=96.38,7921.66");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=183.90,7916.31");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=456.77,7940.90");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=673.80,7931.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=752.96,7946.59");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=262.56,7894.18");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=355.17,7877.13");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=563.83,7883.71");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=788.24,7894.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=936.12,7912.81");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=996.97,7886.58");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=29.84,7870.39");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=128.66,7852.28");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=417.81,7850.64");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=643.97,7869.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=706.18,7865.24");
    }

}

