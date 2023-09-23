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

network DADCA_ACK_n20_u16_a8000m2_3
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
        @display("i=misc/sensor;p=1319.59,7899.07");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3347.98,7904.10");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3824.50,7861.10");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5107.34,7888.32");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5535.46,7799.46");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=869.19,7744.19");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1613.22,7627.07");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1957.34,7530.12");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3379.37,7509.45");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4172.75,7618.38");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4536.33,7740.65");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=356.80,7399.08");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1024.12,7370.88");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2327.15,7410.70");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2867.40,7344.46");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3666.86,7305.21");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4797.30,7484.70");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5173.73,7448.72");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5657.80,7456.25");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1425.37,7214.16");
    }

}

