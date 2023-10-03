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

network DADCA_ACK_n20_u16_a8000m2_6
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
        @display("i=misc/sensor;p=519.21,7927.68");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1185.56,7769.24");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1746.78,7784.16");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2958.63,7978.88");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3370.14,7811.65");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4136.47,7860.87");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4666.46,7937.63");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5361.01,7966.52");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2329.78,7677.30");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2731.65,7538.57");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4514.69,7607.38");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5095.82,7559.49");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5555.12,7667.94");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=214.00,7449.72");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=744.76,7482.20");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1548.56,7490.65");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4199.00,7352.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5431.37,7263.80");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5866.22,7490.15");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=702.51,7102.70");
    }

}

