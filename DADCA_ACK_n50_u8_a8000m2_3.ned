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

network DADCA_ACK_n50_u8_a8000m2_3
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
        @display("i=misc/sensor;p=1603.00,7852.26");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2092.82,7869.17");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2475.73,7910.92");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2896.60,7973.28");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3957.57,7788.00");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5010.13,7983.26");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5633.75,7898.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=611.97,7622.94");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1056.25,7685.30");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3443.69,7646.43");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4473.63,7724.14");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5103.84,7523.04");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=251.20,7424.88");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1464.11,7468.29");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1865.91,7302.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2182.62,7504.41");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2695.59,7330.36");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3884.00,7282.60");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4760.96,7336.45");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5694.31,7452.17");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=715.43,7100.22");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1101.75,7053.00");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=1451.42,7115.40");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3062.55,7222.98");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4376.25,7216.89");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5331.78,7040.21");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=302.47,7005.06");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1894.19,6868.25");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2302.58,6933.75");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3431.70,6989.32");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=172.99,6611.42");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=671.33,6634.55");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1251.91,6674.87");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=2958.55,6751.87");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4019.07,6673.76");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4735.94,6687.69");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=420.48,6322.81");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1008.91,6309.04");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1831.85,6292.47");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2346.82,6305.55");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2668.43,6501.22");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3288.35,6481.16");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3653.52,6483.40");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4399.62,6448.68");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5351.52,6380.97");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2990.77,6165.00");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3621.59,6122.91");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3987.30,6254.70");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=918.71,5949.74");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1273.87,5935.66");
    }

}

