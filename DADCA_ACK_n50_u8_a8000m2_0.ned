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

network DADCA_ACK_n50_u8_a8000m2_0
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
        @display("i=misc/sensor;p=418.45,7772.45");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1414.81,7849.69");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2434.46,7852.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3289.07,7933.64");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3693.10,7972.12");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4113.30,7812.11");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5119.90,7886.30");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2709.21,7542.71");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3080.24,7602.13");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4705.52,7560.79");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=425.87,7364.91");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=837.42,7472.26");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1468.83,7448.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2120.91,7440.88");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3738.77,7433.63");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4319.68,7359.77");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5386.74,7455.80");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=878.91,7109.03");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1751.66,7079.20");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2492.00,7214.61");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3072.87,7031.27");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4778.47,7041.44");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5696.29,7016.12");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=486.37,6938.34");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4058.87,6977.76");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4521.74,6768.43");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1356.30,6743.00");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2230.93,6705.10");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2583.12,6665.25");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3638.57,6746.47");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5361.67,6750.74");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5735.32,6609.71");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=174.46,6441.93");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=705.97,6513.57");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1208.10,6388.45");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1818.41,6472.72");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3184.74,6447.78");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5060.22,6435.06");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=372.30,6054.84");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=853.82,6075.13");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1343.13,6043.09");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2017.93,6164.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2619.95,6234.32");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3477.82,6041.26");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3899.53,6207.40");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4407.07,6027.60");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5394.83,6127.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1853.62,5806.86");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2245.75,5777.45");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2904.32,5998.37");
    }

}

